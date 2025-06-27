#!/usr/bin/env python3
import argparse
import math
import time
import textwrap

import pybullet as p
import pybullet_data

# --- URDF definition for a simple two-link arm ---
simple_urdf = textwrap.dedent("""\
<?xml version="1.0"?>
<robot name="simple_two_link">
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0"
               iyz="0"/>
    </inertial>
    <visual>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  <link name="link1">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0"
               ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><cylinder radius="0.05" length="1"/></geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>
  <link name="link2">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0"
               ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry><cylinder radius="0.05" length="1"/></geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10"
           velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10"
           velocity="1"/>
  </joint>
</robot>
""")


def write_urdf(filename="simple_arm.urdf"):
    """Save the two-link URDF to disk."""
    with open(filename, "w") as f:
        f.write(simple_urdf)
    print(f"Wrote URDF to '{filename}'")


def run_simulation(mode="GUI", steps=1000):
    """Set up PyBullet, load objects, and run a sinusoidal joint demo."""
    # Connect
    if mode.upper() == "GUI":
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    # Search path + gravity
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Ground plane
    plane_id = p.loadURDF("plane.urdf")
    print("Loaded plane id:", plane_id)

    # Write & load the arm URDF
    write_urdf("simple_arm.urdf")
    robot_id = p.loadURDF(
        "simple_arm.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True
    )
    print("Loaded robot id:", robot_id)

    # Introspect joints
    num_joints = p.getNumJoints(robot_id)
    print("Number of joints:", num_joints)
    for ji in range(num_joints):
        info = p.getJointInfo(robot_id, ji)
        jname = info[1].decode()
        clink = info[12].decode()
        print(f" Joint {ji}: name={jname}, child_link={clink}")

    # We'll treat the last link (index=num_joints-1) as the EE
    ee_link = num_joints - 1
    print("Using link", ee_link, "as end-effector")

    # Disable default motors (so we can position-control)
    for ji in range(num_joints):
        p.setJointMotorControl2(
            robot_id, ji,
            controlMode=p.VELOCITY_CONTROL,
            force=0
        )

    # Main sim loop: drive joint1 & joint2 in sine/cosine
    for step in range(steps):
        t = step * (1.0 / 240.0)
        target1 = 0.5 * math.sin(2 * math.pi * 0.5 * t)
        target2 = 0.25 * math.cos(2 * math.pi * 0.5 * t)

        p.setJointMotorControl2(
            robot_id, 0,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target1,
            force=10,
        )
        p.setJointMotorControl2(
            robot_id, 1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target2,
            force=10,
        )

        p.stepSimulation()

        # Print EE position every ~1 s (240 steps)
        if step % 240 == 0:
            st = p.getLinkState(robot_id, ee_link)
            pos = st[0] if st else None
            print(f"Step {step:4d}: EE world pos = {pos}")

        if mode.upper() == "GUI":
            time.sleep(1.0 / 240.0)

    p.disconnect()
    print("Simulation complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test simple two-link arm with moving joints"
    )
    parser.add_argument(
        "--mode",
        choices=["GUI", "DIRECT"],
        default="GUI",
        help="PyBullet connection mode"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=1000,
        help="Number of simulation steps"
    )
    args = parser.parse_args()
    run_simulation(args.mode, args.steps)