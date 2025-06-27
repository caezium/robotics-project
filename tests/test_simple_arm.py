#!/usr/bin/env python3
import argparse
import math
import time

import pybullet as p
import pybullet_data


def create_simple_two_link_urdf(filename="simple_arm.urdf"):
    """Writes a two‐link robot URDF to disk."""
    urdf = """<?xml version="1.0"?>
<robot name="simple_two_link">
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" iyy="1" izz="1"
               ixy="0" ixz="0" iyz="0"/>
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
      <inertia ixx="0.1" iyy="0.1" izz="0.1"
               ixy="0" ixz="0" iyz="0"/>
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
      <inertia ixx="0.1" iyy="0.1" izz="0.1"
               ixy="0" ixz="0" iyz="0"/>
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
    <limit lower="-3.14" upper="3.14"
           effort="10" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14"
           effort="10" velocity="1"/>
  </joint>
</robot>"""
    with open(filename, "w") as f:
        f.write(urdf)
    print(f"Wrote URDF to '{filename}'")


def run_simulation(mode="GUI", num_steps=1000):
    """Connects, loads plane + robot, runs a simple moving test."""
    if mode.upper() == "GUI":
        client = p.connect(p.GUI)
    else:
        client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Ground plane
    plane_id = p.loadURDF("plane.urdf")
    print("Loaded plane id", plane_id)

    # Robot URDF
    create_simple_two_link_urdf("simple_arm.urdf")
    robot_id = p.loadURDF("simple_arm.urdf", basePosition=[0,0,0], useFixedBase=True)
    print("Loaded robot id", robot_id)

    num_joints = p.getNumJoints(robot_id)
    print("Number of joints:", num_joints)
    for ji in range(num_joints):
        info = p.getJointInfo(robot_id, ji)
        joint_name = info[1].decode("utf-8")
        link_name  = info[12].decode("utf-8")
        print(f" Joint {ji}: name={joint_name}, childLink={link_name}")

    # Decide which link is our “end‐effector”
    ee_link_index = num_joints - 1
    print(f"Using link index {ee_link_index} as end–effector")

    # … zero out motors, etc. …

    for step in range(num_steps):
        t = step * (1.0/240.0)
        # set sinusoidal targets …
        p.stepSimulation()

        if step % 240 == 0:
            link_state = p.getLinkState(robot_id, ee_link_index)
            if link_state is None:
                print(f"*** getLinkState returned None for link {ee_link_index}!")
            else:
                pos_world = link_state[0]
                print(f" Step {step:4d}: EE world pos = {pos_world}")

        if mode=="GUI":
            time.sleep(1/240.0)

    p.disconnect()
    print("Simulation finished.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test simple two‐link arm in PyBullet"
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="GUI",
        choices=["GUI", "DIRECT"],
        help="PyBullet connection mode",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=1000,
        help="Number of simulation steps",
    )
    args = parser.parse_args()
    run_simulation(args.mode, args.steps)