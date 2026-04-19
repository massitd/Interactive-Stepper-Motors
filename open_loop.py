#!/usr/bin/env python3
"""
Raspberry Pi: OSC stepper "position chaser" (TouchDesigner -> MotorKit)

TouchDesigner sends:
  /motor{n}/target  [value]
where value is normalized 0.0 .. 1.0

Pi maps normalized -> step target using per-motor ranges, then continuously steps toward it.

Notes:
- This is open-loop positioning (software-tracked step count).
- You should add homing/limit switches if you need true absolute position.
"""

from __future__ import annotations

import asyncio
import threading
import time
from dataclasses import dataclass

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import AsyncIOOSCUDPServer

from adafruit_motor import stepper
from adafruit_motorkit import MotorKit


# -------------------------------
# OSC server config
# -------------------------------
IP = "0.0.0.0"
PORT = 2222

# -------------------------------
# Motor + motion config
# -------------------------------
MOTOR_COUNT = 2  # increase as you add hardware

# Per-motor step ranges: normalized 0..1 maps into [min_steps, max_steps]
# Example: 0.0 -> 0 steps, 1.0 -> 4000 steps
MOTOR_LIMITS = {
    1: (0, 1500),
    2: (0, 1500),
    # 3: (0, 4000),
    # ...
}

# Optional invert per motor (flip direction mapping)
INVERT = {
    1: False,
    2: False,
}

# Motor stepping style
STEP_STYLE = stepper.SINGLE

# Motion policy (all on the Pi)
DEADBAND_STEPS = 1           # within this many steps, consider "at target"
MAX_SPEED = 600            # steps/sec cap (MotorKit + Python: keep realistic)
MIN_SPEED = 40.0             # steps/sec near target
SLOWDOWN_DIST = 150          # steps: within this range, ramp down toward MIN_SPEED

# If True, release coils when idle (less heat, can slip)
RELEASE_WHEN_IDLE = False

# Thread timing
WAKE_TIMEOUT = 0.02
IDLE_SLEEP = 0.01


# -------------------------------
# Helpers
# -------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def parse_motor_num(address: str) -> int | None:
    # Accept addresses like "/motor12/target"
    if not address.startswith("/motor"):
        return None
    tail = address[len("/motor"):]
    digits = []
    for ch in tail:
        if ch.isdigit():
            digits.append(ch)
        else:
            break
    if not digits:
        return None
    try:
        return int("".join(digits))
    except ValueError:
        return None


def norm_to_steps(motor_num: int, norm: float) -> int:
    """Map 0..1 -> [min_steps..max_steps] with optional invert."""
    norm = clamp(norm, 0.0, 1.0)
    if INVERT.get(motor_num, False):
        norm = 1.0 - norm

    mn, mx = MOTOR_LIMITS[motor_num]
    # linear map, rounded to int steps
    return int(round(mn + norm * (mx - mn)))


def speed_from_error(err_steps: int) -> float:
    """Distance-based speed: far -> MAX_SPEED, near -> MIN_SPEED."""
    d = abs(err_steps)
    if d >= SLOWDOWN_DIST:
        return MAX_SPEED
    t = d / float(SLOWDOWN_DIST)  # 0..1
    return MIN_SPEED + (MAX_SPEED - MIN_SPEED) * t


# -------------------------------
# Motor setup
# -------------------------------
kit = MotorKit()
steppers = {i: getattr(kit, f"stepper{i}") for i in range(1, MOTOR_COUNT + 1)}

for i in steppers:
    if i not in MOTOR_LIMITS:
        raise ValueError(f"Missing MOTOR_LIMITS for motor {i}")


# -------------------------------
# State + worker threads
# -------------------------------
@dataclass
class MotorState:
    # What TD last asked for (normalized)
    target_norm: float = 0.0

    # Derived step target (computed in worker for consistency)
    target_steps: int = 0

    # Software-tracked current position in steps
    current_steps: int = 0

    lock: threading.Lock = None
    wake: threading.Event = None


states: dict[int, MotorState] = {}
for i in steppers:
    st = MotorState()
    st.lock = threading.Lock()
    st.wake = threading.Event()
    st.target_steps = norm_to_steps(i, 0.0)
    states[i] = st

shutdown_flag = threading.Event()


def motor_worker(motor_num: int) -> None:
    motor = steppers[motor_num]
    st = states[motor_num]
    next_step_time = time.perf_counter()

    while not shutdown_flag.is_set():
        st.wake.wait(timeout=WAKE_TIMEOUT)
        st.wake.clear()

        # Snapshot state
        with st.lock:
            target_norm = float(st.target_norm)
            cur = int(st.current_steps)

        target = norm_to_steps(motor_num, target_norm)

        # store derived target (optional, for debugging)
        with st.lock:
            st.target_steps = target

        err = target - cur
        if abs(err) <= DEADBAND_STEPS:
            if RELEASE_WHEN_IDLE:
                try:
                    motor.release()
                except Exception:
                    pass
            time.sleep(IDLE_SLEEP)
            continue

        direction = stepper.FORWARD if err > 0 else stepper.BACKWARD

        speed = clamp(speed_from_error(err), 1.0, MAX_SPEED)
        step_interval = 1.0 / speed

        now = time.perf_counter()
        if now < next_step_time:
            time.sleep(min(0.005, next_step_time - now))
            continue

        try:
            motor.onestep(direction=direction, style=STEP_STYLE)
        except Exception as e:
            print(f"[motor{motor_num}] onestep error: {e}")
            time.sleep(0.05)
            continue

        with st.lock:
            st.current_steps += (1 if direction == stepper.FORWARD else -1)

        next_step_time = now + step_interval


# Start workers
for m in steppers:
    threading.Thread(target=motor_worker, args=(m,), daemon=True).start()


# -------------------------------
# OSC handler
# -------------------------------
def handle_target(address: str, *args) -> None:
    if not address.endswith("/target"):
        return

    motor_num = parse_motor_num(address)
    if motor_num is None or motor_num not in states:
        return

    if len(args) < 1:
        return

    try:
        norm = float(args[0])
    except Exception:
        return

    # Clamp here too (cheap + defensive)
    norm = clamp(norm, 0.0, 1.0)

    st = states[motor_num]
    with st.lock:
        st.target_norm = norm
    st.wake.set()


# -------------------------------
# OSC server
# -------------------------------
dispatcher = Dispatcher()
dispatcher.set_default_handler(handle_target)


async def main() -> None:
    loop = asyncio.get_event_loop()
    server = AsyncIOOSCUDPServer((IP, PORT), dispatcher, loop)
    transport, _ = await server.create_serve_endpoint()
    print(f"OSC listening on {IP}:{PORT} (expects /motorN/target [0..1])")

    try:
        await asyncio.Event().wait()
    finally:
        transport.close()


# -------------------------------
# Run
# -------------------------------
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    shutdown_flag.set()
    for st in states.values():
        st.wake.set()
    for m in steppers.values():
        try:
            m.release()
        except Exception:
            pass
