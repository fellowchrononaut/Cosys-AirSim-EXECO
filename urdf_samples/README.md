# urdf_samples

Sample clients for the `urdfbot` vehicle type — arbitrary URDF robots on a Box3D backend.

These talk to the simulator over RPC on **port 41454**. MultiAgent runs one server per vehicle
family (41451 drone, 41452 car, 41453 computer vision, 41454 urdfbot) because each exposes a
different control surface; connecting to the wrong one succeeds and then fails on the first call.

## urdfbot_jog.py

A jog panel: drive every joint of a URDF robot by hand and watch what it actually does.

```
python3 urdf_samples/urdfbot_jog.py --settings ~/Documents/AirSim/settings.json
```

Point `--settings` at the settings file **the simulator is running** — the same one chosen in the
"custom settings.json" dialog on Play. It is read only to learn which vehicles are of type
`urdfbot`.

Each commandable joint gets a row: name, type, control mode, a slider, the command you gave, and
the position and velocity the simulator reports. A vehicle dropdown switches between robots.

The **p/v** column compares the direction the reported position is travelling against the sign of
the reported velocity and shows `ok` or `MISMATCH`. It is not decoration: that disagreement is what
exposed a joint-axis bug in which every hinge perpendicular to Z ran backwards, because
`position` came from the physics engine's joint frame while `velocity` was derived by projecting
onto the URDF `<axis>`. One struct, two conventions, and only their disagreement made it visible.

### Things worth knowing before you use it

- **Joint mode and Drive mode are exclusive.** `setDriveCommand` *latches*: once issued, the drive
  loop writes every drive joint's velocity target every physics step and silently overwrites
  anything `setJointVelocity` does afterwards — the joint call still returns success. Switching
  back to joint mode calls `enableApiControl(True)`, which is the only way to clear that latch.
- **Only commandable joints appear.** `fixed` joints have no state, and a `<mimic>` joint resolved
  cosmetically has no solver joint at all, so commanding it does nothing. A slider that does
  nothing is worse than an absent one.
- **Commands latch.** A joint told to spin keeps spinning; there is no watchdog on this path. STOP
  zeroes everything, Release hands control back to the keyboard, and closing the window does both
  for every vehicle.
- **Never pass an empty vehicle name to a family server**, and never address a vehicle whose type
  you have not confirmed — see the note at the top of the script. Vehicle types are stated only in
  the settings JSON; there is no RPC that reports them.

### Requirements

Python 3 with `tkinter` (standard library) and `msgpack-rpc-python`. The `cosysairsim` client is
imported from `../PythonClient` by path, so a fresh checkout needs no install step.
