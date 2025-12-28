from pythonosc.osc_server import AsyncIOOSCUDPServer
from pythonosc.dispatcher import Dispatcher
import asyncio

import board 
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
import threading
import time


# -------------------------------
# Motor setup
# -------------------------------
kit = MotorKit()
motor_count = 2

# map steppers
steppers = {}
for i in range (1, motor_count + 1):
    steppers[i] = getattr(kit, f"stepper{i}")

# per-motor stop flags
stop_flags = {motor_num: False for motor_num in steppers}

# -------------------------------
# Motor control functions
# -------------------------------
def move_motor(motor_num, direction, steps, style=stepper.SINGLE, delay=0.01):
    motor = steppers[motor_num]
    for _ in range(steps):
        if stop_flags[motor_num]:
            break
        motor.onestep(direction=direction, style=style)
        time.sleep(delay)

def move_motor_threaded(motor_num, direction, steps, style=stepper.SINGLE, delay=0.01):
    threading.Thread(target=move_motor, args=(motor_num, direction, steps, style, delay), daemon=True).start()

def stop_motor(motor_num):
    stop_flags[motor_num] = True

def reset_stop_flag(motor_num):
    stop_flags[motor_num] = False
    
# -------------------------------
# OSC handling
# -------------------------------
def parse_motor_address(address):
    # extract motor number from OSC address
    try:
        return int(address.replace("/motor", ""))
    except ValueError:
        return None

def parse_motor_args(args):
    # convert osc args into direction, steps, style and delay
    # expects at least [direction, steps] optional: [style, delay]
    direction = stepper.FORWARD if args[0] == "forward" else stepper.BACKWARD
    steps = int(args[1])
    style = getattr(stepper, args[2].upper(), stepper.SINGLE) if len(args) >= 3 else stepper.SINGLE
    delay = float(args[3]) if len(args) >= 4 else 0.01

    return direction, steps, style, delay

def filter_handler(address, *args):
    motor_num = parse_motor_address(address)
    # check motor validity
    if motor_num not in steppers:
        print(f"invalid motor: {address}")
        return
    # stop motor
    if len(args) == 1 and args[0] == "stop":
        stop_motor(motor_num)
        print(f"Motor {motor_num} stopped")
        return
    # check argument validity
    result = parse_motor_args(args)
    if result is None:
        print(f"Invalid args for {address}: {args}")
        return

    direction, steps, style, delay = result
    reset_stop_flag(motor_num)
    move_motor_threaded(motor_num, direction, steps, style, delay)

    print(f"{address}: {args}")

# -------------------------------
# OSC server setup
# -------------------------------
ip = "0.0.0.0"

port = 2222

dispatcher = Dispatcher()
for i in range(1, motor_count + 1):
    dispatcher.map(f"/motor{i}", filter_handler)


async def loop():
    await asyncio.sleep(1)

async def init_main():
    server = AsyncIOOSCUDPServer((ip, port), dispatcher, asyncio.get_event_loop())
    print ("Serving on {}".format(server._server_address))
    transport, protocol = await server.create_serve_endpoint()

    try:
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        pass
        
# -------------------------------
# Run
# -------------------------------
try:
    asyncio.run(init_main())
except KeyboardInterrupt:
    print("Shutting down OSC Server.")