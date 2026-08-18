"""Python client for the URDF robot API.

Joint-level and nothing more, mirroring UrdfBotApiBase. A URDF says nothing about which of its
joints are wheels, which steer, or which belong to an arm — so higher-level control (Ackermann,
differential drive, IK) belongs in the caller, where the robot's semantics are known. Guessing it
here would be the silent-wrong-answer this workstream exists to avoid.

⚠ Port 41454, not 41451. MultiAgent runs one server per vehicle family — 41451 drone, 41452 car,
41453 computer vision, 41454 urdfbot — because each exposes a different control surface. Connecting
to the wrong port succeeds and then fails on the first call, which is a confusing way to find out.
"""

import msgpackrpc  # pip install rpc-msgpack


class UrdfBotClient:
    def __init__(self, ip="", port=41454, timeout_value=3600):
        if ip == "":
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(
            msgpackrpc.Address(ip, port),
            timeout=timeout_value,
            pack_encoding='utf-8',
            unpack_encoding='utf-8',
        )

    def ping(self):
        return self.client.call('ping')

    # ---------------------------------------------------------------- control handover
    def enableApiControl(self, is_enabled, vehicle_name=''):
        """Take control away from the keyboard, or hand it back.

        ⚠ **Call this with True before commanding anything.** The keyboard drive loop writes every
        drive and steer joint on every physics step, so with it active an RPC command is
        overwritten 3 ms after it is issued — the call succeeds, returns cleanly, and the robot
        does not move. Starts False, so the keyboard works on a freshly loaded robot.
        """
        self.client.call('enableApiControl', is_enabled, vehicle_name)

    def isApiControlEnabled(self, vehicle_name=''):
        return self.client.call('isApiControlEnabled', vehicle_name)

    # ---------------------------------------------------------------- structure
    def getJoints(self, vehicle_name=''):
        """Every joint the robot declares.

        Each entry carries ``mimic_role``: "none", "cosmetic" or "load-bearing". ⚠ A **cosmetic**
        mimic joint has NO solver joint behind it — it is resolved by forward kinematics — so
        commanding it does nothing. That is reported rather than hidden, because a command that is
        silently ignored is worse than one that is refused.
        """
        return self.client.call('getJoints', vehicle_name)

    def getLinkNames(self, vehicle_name=''):
        return self.client.call('getLinkNames', vehicle_name)

    # ---------------------------------------------------------------- command
    def setJointPosition(self, joint, radians_or_metres, vehicle_name=''):
        """Position control: a spring drives the joint to the target, motor off.

        ⚠ Steady-state accuracy is bounded by the open M4 gap — there is no calibrated
        kp-to-hertz conversion and no gravity-compensation feedforward — so a gravity-loaded arm
        will not settle exactly. A rover's steering joints, which is what this was built for, track
        fine. See NEXT-SESSION.md section 6.
        """
        self.client.call('setJointPosition', joint, float(radians_or_metres), vehicle_name)

    def setJointVelocity(self, joint, per_second, vehicle_name=''):
        self.client.call('setJointVelocity', joint, float(per_second), vehicle_name)

    def setDriveCommand(self, throttle, steering, vehicle_name=''):
        """Drive with the same two axes as the keyboard, each in [-1, 1].

        throttle scales UrdfDrive.MaxWheelSpeed across DriveJoints; steering scales
        MaxSteerAngle across SteerJoints, each by its per-joint multiplier.

        ⚠ Requires enableApiControl(True) first. Without it the keyboard remains the input
        source and this command is silently ignored — the call still succeeds.

        ⚠ Issuing this LATCHES drive control: from the first drive command the drive loop
        writes every drive and steer joint each physics step, so setJointPosition/Velocity/Effort
        on those particular joints will no longer hold. Other joints are unaffected.

        To go back to joint-level control of the wheels, toggle enableApiControl (False then
        True) — that clears the latch and zeroes the axes.
        """
        self.client.call('setDriveCommand', float(throttle), float(steering), vehicle_name)

    def getUrdfXml(self, vehicle_name=''):
        """The robot's URDF as the simulator loaded it.

        ⚠ Prefer this over opening UrdfFile yourself. A client in a container generally cannot
        see that path — verified for the ROS 2 container, where only one host directory is
        mounted. It is also the exact text this simulator parsed, so a description and the joint
        names it is matched against cannot drift apart.
        """
        return self.client.call('getUrdfXml', vehicle_name)

    def getJointStates(self, vehicle_name=''):
        """Every movable joint's state in one call, sampled together.

        Returns a list of dicts: name, position, velocity, effort.

        ⚠ Prefer this over calling getJointState per joint. N separate calls sample N different
        instants, so the result is a smear rather than a snapshot — which matters when it feeds a
        sensor_msgs/JointState and from there robot_state_publisher's TF.

        ⚠ FIXED joints are absent (no state to report), and so are COSMETIC <mimic> joints, which
        have no solver joint at all. Their coupling is visible via getJoints()'s mimic_role.
        """
        return self.client.call('getJointStates', vehicle_name)

    def setJointEffort(self, joint, newton_metres, vehicle_name=''):
        """⚠ Box3D has no direct torque input on a revolute joint. Effort is expressed as an
        unreachable speed capped by the requested torque, which is the standard idiom — but it is
        an idiom, not a torque source."""
        self.client.call('setJointEffort', joint, float(newton_metres), vehicle_name)

    def setJointTorques(self, joints, newton_metres, vehicle_name=''):
        """Every joint's torque in ONE call, applied together.

        ⚠ Use this, not N setJointEffort calls, for anything running a control loop. A legged
        controller is ``tau = kp*(q* - q) - kd*qd`` evaluated on ONE snapshot of the robot; the
        torques it produces describe a single instant. Sent separately they land at N different
        instants, each on a slightly different robot — the actuation mirror of the smear that
        getJointStates exists to avoid. At a quadruped's 50 Hz it is also 600 round trips a second
        rather than 50.

        ⚠ Torque is CLAMPED per joint to the URDF's <limit effort>, the ceiling the real actuator
        could produce. getJointStates reports the clamped value back as ``effort``.

        ⚠ All-or-nothing: a name the robot does not have raises, and no torque is applied. A
        controller silently driving eleven of twelve legs is the failure mode this avoids.
        """
        self.client.call('setJointTorques', list(joints),
                         [float(v) for v in newton_metres], vehicle_name)

    def setJointPositionGains(self, joint, hertz, damping_ratio, vehicle_name=''):
        """⚠ Keep hertz below half the physics rate. The sim steps at 3 ms, so the Nyquist ceiling
        is about 166 Hz; 40 Hz with damping 1.0 is what the rover's steering uses."""
        self.client.call('setJointPositionGains', joint, float(hertz), float(damping_ratio),
                         vehicle_name)

    # ---------------------------------------------------------------- state
    def getJointState(self, joint, vehicle_name=''):
        """position, velocity, effort. ``effort`` is the motor's applied effort, not the
        constraint reaction — those differ and the distinction is preserved deliberately."""
        return self.client.call('getJointState', joint, vehicle_name)

    def getUrdfBotState(self, vehicle_name=''):
        """The robot's kinematics AND the simulator time they were sampled at, as one value.

        Returns a dict: ``kinematics_estimated`` (position, orientation, linear/angular velocity
        and acceleration) and ``timestamp`` in nanoseconds on the simulator clock — the same clock
        CarState.timestamp and MultirotorState.timestamp use, so stamps compare directly across
        vehicle types.

        ⚠ Use this, not simGetGroundTruthKinematics, for anything you intend to timestamp. The
        kinematics are the same; what that call cannot give you is the time. Reading a clock
        separately stamps the sample with an instant it was not taken at, and under a paused or
        scaled clock that gap is unbounded rather than one round-trip.

        ⚠ Position is relative to this vehicle's own spawn point while orientation is absolute —
        the hybrid frame every AirSim vehicle reports, not something this call introduces.
        """
        return self.client.call('getUrdfBotState', vehicle_name)

    def getLinkPose(self, link, vehicle_name=''):
        """Pose and twist of a link, in **AirSim NED** like every other pose crossing this API —
        not in the URDF's own frame. The conversion happens simulator-side, so a script that
        already handles drones and cars needs no special case here."""
        return self.client.call('getLinkPose', link, vehicle_name)
