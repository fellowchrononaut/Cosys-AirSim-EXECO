#!/usr/bin/env python3
"""A jog panel for URDF robots: drive every joint by hand and watch what it actually does.

Built to answer one question directly — "does a POSITIVE command move this joint the way the URDF
says it should?" — for each joint individually, rather than inferring it from how the whole robot
behaves. Each row shows the command you gave, the position the sim reports, and the velocity the
sim reports, side by side.

    python3 urdf_samples/urdfbot_jog.py --settings ~/Documents/AirSim/urdfbot_scout_exomy.json

The settings file is the one the SIMULATOR IS RUNNING — the same file you chose in the "custom
settings.json" dialog on Play. It is read here only to learn which vehicles are of type "urdfbot";
nothing else in it is used.

⚠ VEHICLE NAMES COME FROM THE SETTINGS FILE, never from simListVehicles. Every family RPC server
(41451 drone / 41452 car / 41453 computer vision / 41454 urdfbot) resolves a name through ONE
shared ApiProvider and then static_casts the result to its own family type, unchecked. Asking the
urdfbot server about a car therefore does not raise — it SIGSEGVs the whole simulator. An empty
vehicle name is the usual way to trip this, because it resolves to the scene's first vehicle
whatever its type. The settings file is the only place vehicle TYPES are stated.

⚠ DRIVE MODE AND JOINT MODE ARE EXCLUSIVE, and this is not a UI nicety. setDriveCommand LATCHES in
the sim: once issued, the drive loop writes every drive joint's velocity target every physics step
and silently overwrites anything setJointVelocity does afterwards. The joint call still returns
success. Switching back to joint mode calls enableApiControl(True), which clears that latch — the
only way to release it.

⚠ Commands LATCH. A joint told to spin keeps spinning until told otherwise; there is no watchdog on
this path. STOP zeroes everything, and closing the window does the same and hands control back to
the keyboard.
"""

import argparse
import json
import os
import sys
import tkinter as tk
from tkinter import ttk

HERE = os.path.dirname(os.path.abspath(__file__))
# The client library ships in this repo, one level up. Imported by path rather than requiring an
# installed cosysairsim, so the sample runs from a fresh checkout with nothing to set up.
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'PythonClient'))

from cosysairsim.urdfbot import UrdfBotClient  # noqa: E402

POLL_MS = 100


def urdfbots_from_settings(path):
    with open(path) as fh:
        cfg = json.load(fh)
    return [n for n, v in cfg.get('Vehicles', {}).items()
            if str(v.get('VehicleType', '')).lower() == 'urdfbot']


class JointRow:
    def __init__(self, parent, row, client, vehicle, info, limits):
        self.c, self.v, self.name = client, vehicle, info['name']
        self.type = info['type']
        self.limits = limits

        ttk.Label(parent, text=self.name, width=20, anchor='w').grid(row=row, column=0, sticky='w')
        ttk.Label(parent, text=self.type, width=11, anchor='w',
                  foreground='#666').grid(row=row, column=1, sticky='w')

        # Position control for a joint with limits, velocity for a continuous one. That matches how
        # the joint is actually used: a wheel has no meaningful target angle, a steering joint has
        # no meaningful steady speed.
        default = 'position' if (self.type != 'continuous' and limits) else 'velocity'
        self.mode = tk.StringVar(value=default)
        ttk.OptionMenu(parent, self.mode, default, 'position', 'velocity', 'effort',
                       command=lambda _=None: self._rescale()).grid(row=row, column=2, padx=4)

        self.val = tk.DoubleVar(value=0.0)
        self.scale = ttk.Scale(parent, from_=-1, to=1, variable=self.val, length=230,
                               command=lambda _=None: self._send())
        self.scale.grid(row=row, column=3, padx=6)

        self.cmd_lbl = ttk.Label(parent, text='0.000', width=8, anchor='e')
        self.cmd_lbl.grid(row=row, column=4)
        self.pos_lbl = ttk.Label(parent, text='--', width=9, anchor='e')
        self.pos_lbl.grid(row=row, column=5)
        self.vel_lbl = ttk.Label(parent, text='--', width=9, anchor='e')
        self.vel_lbl.grid(row=row, column=6)
        # ⚠ Position and velocity disagreeing about the direction of travel is a REAL failure mode,
        # not a rounding artifact: it is what exposed the Box3D joint-axis bug on 2026-08-18. This
        # column calls it out instead of leaving it to be noticed.
        self.agree_lbl = ttk.Label(parent, text='', width=7, anchor='center')
        self.agree_lbl.grid(row=row, column=7)

        ttk.Button(parent, text='0', width=3,
                   command=self.zero).grid(row=row, column=8, padx=(4, 0))
        self._rescale()
        self._last_pos = None

    def _rescale(self):
        m = self.mode.get()
        if m == 'position' and self.limits:
            lo, hi = self.limits
        elif m == 'position':
            lo, hi = -3.14, 3.14
        elif m == 'velocity':
            lo, hi = -6.0, 6.0
        else:
            lo, hi = -20.0, 20.0
        self.scale.configure(from_=lo, to=hi)
        self.zero()

    def zero(self):
        self.val.set(0.0)
        self._send()

    def _send(self):
        v = float(self.val.get())
        self.cmd_lbl.configure(text=f'{v:+.3f}')
        try:
            m = self.mode.get()
            if m == 'position':
                self.c.setJointPosition(self.name, v, self.v)
            elif m == 'velocity':
                self.c.setJointVelocity(self.name, v, self.v)
            else:
                self.c.setJointEffort(self.name, v, self.v)
        except Exception:
            pass

    def update(self, st):
        if st is None:
            self.pos_lbl.configure(text='--')
            self.vel_lbl.configure(text='--')
            self.agree_lbl.configure(text='')
            return
        pos, vel = st['position'], st['velocity']
        self.pos_lbl.configure(text=f'{pos:+.3f}')
        self.vel_lbl.configure(text=f'{vel:+.3f}')

        moving = self._last_pos is not None and abs(pos - self._last_pos) > 1e-4
        if moving and abs(vel) > 0.05:
            d = (pos - self._last_pos)
            ok = (d > 0) == (vel > 0)
            self.agree_lbl.configure(text='ok' if ok else 'MISMATCH',
                                     foreground='#177245' if ok else '#b00020')
        elif abs(vel) < 0.05:
            self.agree_lbl.configure(text='')
        self._last_pos = pos


class App:
    def __init__(self, root, client, vehicles):
        self.c, self.vehicles = client, vehicles
        self.rows = []
        root.title('URDF robot jog panel')

        top = ttk.Frame(root, padding=8)
        top.pack(fill='x')
        ttk.Label(top, text='Vehicle').pack(side='left')
        self.veh = tk.StringVar(value=vehicles[0])
        ttk.OptionMenu(top, self.veh, vehicles[0], *vehicles,
                       command=lambda _=None: self.load()).pack(side='left', padx=6)

        self.mode = tk.StringVar(value='joint')
        ttk.Radiobutton(top, text='Joint control', variable=self.mode, value='joint',
                        command=self.switch_mode).pack(side='left', padx=(18, 4))
        ttk.Radiobutton(top, text='Drive (throttle/steer)', variable=self.mode, value='drive',
                        command=self.switch_mode).pack(side='left')

        ttk.Button(top, text='STOP', command=self.stop_all).pack(side='right')
        ttk.Button(top, text='Release to keyboard',
                   command=self.release).pack(side='right', padx=6)

        hdr = ttk.Frame(root, padding=(8, 0))
        hdr.pack(fill='x')
        for i, (t, w) in enumerate([('joint', 20), ('type', 11), ('mode', 11), ('command', 32),
                                    ('cmd', 8), ('pos', 9), ('vel', 9), ('p/v', 7)]):
            ttk.Label(hdr, text=t, width=w, anchor='w',
                      font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=i, sticky='w')

        self.body = ttk.Frame(root, padding=8)
        self.body.pack(fill='both', expand=True)

        self.drive = ttk.Frame(root, padding=8)
        self.throttle = tk.DoubleVar(value=0.0)
        self.steering = tk.DoubleVar(value=0.0)
        for r, (name, var) in enumerate((('throttle', self.throttle), ('steering', self.steering))):
            ttk.Label(self.drive, text=name, width=10).grid(row=r, column=0)
            ttk.Scale(self.drive, from_=-1, to=1, variable=var, length=320,
                      command=lambda _=None: self.send_drive()).grid(row=r, column=1, padx=6)
            ttk.Button(self.drive, text='0', width=3,
                       command=lambda v=var: (v.set(0.0), self.send_drive())).grid(row=r, column=2)
        ttk.Label(self.drive, foreground='#666',
                  text='Right / D bind to steering = +1, so positive should turn RIGHT.'
                  ).grid(row=2, column=0, columnspan=3, pady=(6, 0), sticky='w')

        self.status = ttk.Label(root, text='', padding=(8, 4), foreground='#555')
        self.status.pack(fill='x')

        self.load()
        root.protocol('WM_DELETE_WINDOW', lambda: (self.shutdown(), root.destroy()))
        root.after(POLL_MS, self.poll)

    def load(self):
        for w in self.body.winfo_children():
            w.destroy()
        self.rows = []
        v = self.veh.get()
        try:
            joints = self.c.getJoints(v)
            commandable = {j['name'] for j in self.c.getJointStates(v)}
        except Exception as e:
            self.status.configure(text=f'cannot read {v}: {e}')
            return
        r = 0
        for j in joints:
            # ⚠ Only joints the sim will actually report state for. A FIXED joint has none, and a
            # COSMETIC <mimic> has no solver joint at all — commanding it does nothing, and a
            # slider that does nothing is worse than an absent one.
            if j['name'] not in commandable:
                continue
            lim = (j['lower'], j['upper']) if j.get('has_limit') else None
            self.rows.append(JointRow(self.body, r, self.c, v, j, lim))
            r += 1
        self.status.configure(
            text=f'{v}: {len(self.rows)} commandable joint(s) of {len(joints)} '
                 f'(fixed and cosmetic-mimic joints are omitted)')
        self.switch_mode()

    def switch_mode(self):
        v = self.veh.get()
        try:
            # Clears the drive latch. Without this, joint commands are overwritten every physics
            # step by a drive command issued earlier and appear to do nothing.
            self.c.enableApiControl(True, v)
        except Exception:
            pass
        if self.mode.get() == 'drive':
            self.body.pack_forget()
            self.drive.pack(fill='x')
        else:
            self.drive.pack_forget()
            self.body.pack(fill='both', expand=True)
            self.throttle.set(0.0)
            self.steering.set(0.0)

    def send_drive(self):
        try:
            self.c.setDriveCommand(float(self.throttle.get()), float(self.steering.get()),
                                   self.veh.get())
        except Exception:
            pass

    def stop_all(self):
        v = self.veh.get()
        try:
            self.c.setDriveCommand(0.0, 0.0, v)
        except Exception:
            pass
        self.throttle.set(0.0)
        self.steering.set(0.0)
        self.c.enableApiControl(True, v)   # clear the latch we just set
        for row in self.rows:
            row.zero()
        self.status.configure(text=f'{v}: all commands zeroed')

    def release(self):
        self.stop_all()
        try:
            self.c.enableApiControl(False, self.veh.get())
        except Exception:
            pass
        self.status.configure(text=f'{self.veh.get()}: API control released, keyboard has it')

    def shutdown(self):
        for v in self.vehicles:
            try:
                self.c.setDriveCommand(0.0, 0.0, v)
                self.c.enableApiControl(True, v)
                for j in self.c.getJointStates(v):
                    self.c.setJointVelocity(j['name'], 0.0, v)
                self.c.enableApiControl(False, v)
            except Exception:
                pass

    def poll(self, root=None):
        v = self.veh.get()
        try:
            st = {j['name']: j for j in self.c.getJointStates(v)}
        except Exception:
            st = {}
        for row in self.rows:
            row.update(st.get(row.name))
        self.body.after(POLL_MS, self.poll)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settings', required=True,
                    help='the settings JSON the sim is running - the only source of vehicle types')
    args = ap.parse_args()
    path = os.path.abspath(os.path.expanduser(args.settings))
    if not os.path.isfile(path):
        print(f'no such settings file: {path}')
        return 1

    vehicles = urdfbots_from_settings(path)
    if not vehicles:
        print(f'{path} declares no urdfbot vehicles')
        return 1

    c = UrdfBotClient(timeout_value=10)
    c.ping()
    print(f'connected (41454); urdfbots: {vehicles}')

    root = tk.Tk()
    App(root, c, vehicles)
    root.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
