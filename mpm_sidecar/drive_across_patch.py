#!/usr/bin/env python3
"""Drive Rover1 straight across the MPM patch over RPC, so the selection test is repeatable.

⚠ API CONTROL IS TAKEN AND ALWAYS RELEASED. Without enableApiControl the pawn's keyboard axes win
and setDriveCommand is a silent no-op; leaving it on afterwards leaves the operator with dead keys.
"""
import sys, time
import msgpackrpc

VEHICLE = "Rover1"

def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
    throttle = float(sys.argv[2]) if len(sys.argv) > 2 else 0.30
    # ⚠ PORT 41454, NOT 41451. MultiAgent mode runs FOUR rpc servers, one per vehicle family:
    #   41451 multirotor · 41452 car · 41453 computer-vision · 41454 urdfbot
    # They are separate on purpose — getVehicleApi() static_casts to the family's api type, so a
    # urdfbot call answered by the multirotor server would cast a UrdfBotApiBase to a MultirotorApi.
    # 41451 answers "could not find function setDriveCommand", which is the polite version of that.
    c = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 41454), timeout=30)
    print("ping:", c.call("ping"))
    c.call("enableApiControl", True, VEHICLE)
    try:
        t0 = time.time()
        while time.time() - t0 < seconds:
            c.call("setDriveCommand", throttle, 0.0, VEHICLE)
            time.sleep(0.05)
        c.call("setDriveCommand", 0.0, 0.0, VEHICLE)
        time.sleep(1.5)
        print(f"drove {seconds:.1f}s at throttle {throttle}")
    finally:
        c.call("setDriveCommand", 0.0, 0.0, VEHICLE)
        c.call("enableApiControl", False, VEHICLE)
        print("api control released")

if __name__ == "__main__":
    raise SystemExit(main())
