#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Float64
from geometry_msgs.msg import Vector3, Pose
from nav_msgs.msg import Odometry
import threading
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import math

StartPos = Vector3()

GATE_POS = (5, 2, 2.48)
FLARE_POS = (5, 4, 2.25)

# publishers
orientation_pub = rospy.Publisher("/emulation/orientation", Vector3, queue_size=1)
actual_orientation_pub = rospy.Publisher(
    "/diagnostics/orientation", Vector3, queue_size=1
)

depth_pub = rospy.Publisher("/depth_data", Float64, queue_size=1)
depth_data = Float64()

thrust_control = rospy.Publisher("/control/thrusters", Float64MultiArray, queue_size=1)
thrust_control_data = Float64MultiArray()

gate_localization_pub = rospy.Publisher("/localization/gate", Vector3, queue_size=1)
flare_localization_pub = rospy.Publisher("/localization/flare", Vector3, queue_size=1)
state = {"hull_front_pose": None, "hull_back_pose": None, "hull_back_right_pose": None}


def on_thrust(x: Float32MultiArray):
    thrust_control_data.data = [a / 100 for a in x.data]
    thrust_control.publish(thrust_control_data)


def on_depth(x: FluidPressure):
    depth_data.data = x.fluid_pressure / (9.8 * 997)
    depth_pub.publish(depth_data)


def on_hull_front(x: Odometry):
    state["hull_front_pose"] = x.pose.pose


def on_hull_back(x: Odometry):
    state["hull_back_pose"] = x.pose.pose


def on_hull_back_right(x: Odometry):
    state["hull_back_right_pose"] = x.pose.pose


# h vector
def calculate_hull_vector() -> np.ndarray:
    hull_front_pose = state["hull_front_pose"]
    hull_back_pose = state["hull_back_pose"]

    if hull_front_pose is None or hull_back_pose is None:
        return None

    front_position = np.array(
        [
            hull_front_pose.position.x,
            hull_front_pose.position.y,
            hull_front_pose.position.z,
        ]
    )
    back_position = np.array(
        [
            hull_back_pose.position.x,
            hull_back_pose.position.y,
            hull_back_pose.position.z,
        ]
    )

    hull_vector = front_position - back_position
    return hull_vector


# g vector
def calculate_gate_vector() -> np.ndarray:
    hull_front_pose = state["hull_front_pose"]

    if hull_front_pose is None:
        return None

    gate_position = np.array(GATE_POS)
    front_position = np.array(
        [
            hull_front_pose.position.x,
            hull_front_pose.position.y,
            hull_front_pose.position.z,
        ]
    )

    gate_vector = gate_position - front_position
    return gate_vector

# f vector
def calculate_flare_vector() -> np.ndarray:
    hull_front_pose = state["hull_front_pose"]

    if hull_front_pose is None:
        return None

    flare_position = np.array(FLARE_POS)
    front_position = np.array(
        [
            hull_front_pose.position.x,
            hull_front_pose.position.y,
            hull_front_pose.position.z,
        ]
    )

    flare_vector = flare_position - front_position
    return flare_vector

# p vector


def calculate_hull_back_right_vector():
    hull_back_pose = state["hull_back_pose"]
    hull_back_right_pose = state["hull_back_right_pose"]

    if hull_back_pose is None or hull_back_right_pose is None:
        return None

    back_position = np.array(
        [
            hull_back_pose.position.x,
            hull_back_pose.position.y,
            hull_back_pose.position.z,
        ]
    )
    back_right_position = np.array(
        [
            hull_back_right_pose.position.x,
            hull_back_right_pose.position.y,
            hull_back_right_pose.position.z,
        ]
    )

    hull_back_right_vector = back_right_position - back_position
    return hull_back_right_vector


def gate_angle() -> float:
    hull_vector = calculate_hull_vector()
    gate_vector = calculate_gate_vector()

    if hull_vector is None or gate_vector is None:
        return 180

    hull_norm = np.linalg.norm(hull_vector)
    gate_norm = np.linalg.norm(gate_vector)

    if hull_norm == 0.0 or gate_norm == 0.0:
        return 0.0

    dot_product = np.dot(hull_vector, gate_vector)
    angle = np.arccos(dot_product / (hull_norm * gate_norm))

    return 180 - math.degrees(angle)

def flare_angle() -> float:
    hull_vector = calculate_hull_vector()
    flare_vector = calculate_flare_vector()

    if hull_vector is None or flare_vector is None:
        return 180

    hull_norm = np.linalg.norm(hull_vector)
    flare_norm = np.linalg.norm(flare_vector)

    if hull_norm == 0.0 or flare_norm == 0.0:
        return 0.0

    dot_product = np.dot(hull_vector, flare_vector)
    angle = np.arccos(dot_product / (hull_norm * flare_norm))

    return 180 - math.degrees(angle)

def calculate_transformed_gate_vector():
    hf_vector = calculate_hull_vector()
    gate_vector = calculate_gate_vector()
    hr_vector = calculate_hull_back_right_vector()

    if hf_vector is None or gate_vector is None or hr_vector is None:
        return None

    p_vector = np.cross(hr_vector, hf_vector)

    basis = (
        hr_vector / np.linalg.norm(hr_vector),
        -hf_vector / np.linalg.norm(hf_vector),
        -p_vector / np.linalg.norm(p_vector),
    )

    return np.linalg.inv(np.column_stack(basis)).dot(gate_vector)

def calculate_transformed_flare_vector():
    hf_vector = calculate_hull_vector()
    flare_vector = calculate_flare_vector()
    hr_vector = calculate_hull_back_right_vector()

    if hf_vector is None or flare_vector is None or hr_vector is None:
        return None

    p_vector = np.cross(hr_vector, hf_vector)

    basis = (
        hr_vector / np.linalg.norm(hr_vector),
        -hf_vector / np.linalg.norm(hf_vector),
        -p_vector / np.linalg.norm(p_vector),
    )

    return np.linalg.inv(np.column_stack(basis)).dot(flare_vector) 


def report_gate_pos():
    gate_vector = calculate_gate_vector()
    transformed_gate_vector = calculate_transformed_gate_vector()
    gate_angle_deg = gate_angle()

    # print(gate_vector, gate_angle_deg, transformed_gate_vector)

    if (
        transformed_gate_vector is not None
        and gate_angle_deg <= 40
        and 0.5 < np.linalg.norm(gate_vector) < 7.5
    ):
        gate_pos = Vector3()

        noise_mean = 0.0  # Mean of the noise
        noise_std = 0.1  # Standard deviation of the noise

        gate_pos.x = transformed_gate_vector[0] + np.random.normal(
            noise_mean, noise_std
        )
        gate_pos.y = transformed_gate_vector[1] + np.random.normal(
            noise_mean, noise_std
        )
        gate_pos.z = transformed_gate_vector[2] + np.random.normal(
            noise_mean, noise_std
        )

        gate_localization_pub.publish(gate_pos)

def report_flare_pos():
    flare_vector = calculate_flare_vector()
    transformed_flare_vector = calculate_transformed_flare_vector()
    flare_angle_deg = flare_angle()

    # print(flare_vector, flare_angle_deg, transformed_flare_vector)

    if (
        transformed_flare_vector is not None
        and flare_angle_deg <= 40
        and 0.5 < np.linalg.norm(flare_vector) < 7.5
    ):
        flare_pos = Vector3()

        noise_mean = 0.0  # Mean of the noise
        noise_std = 0.1  # Standard deviation of the noise

        flare_pos.x = transformed_flare_vector[0] + np.random.normal(
            noise_mean, noise_std
        )
        flare_pos.y = transformed_flare_vector[1] + np.random.normal(
            noise_mean, noise_std
        )
        flare_pos.z = transformed_flare_vector[2] + np.random.normal(
            noise_mean, noise_std
        )

        flare_localization_pub.publish(flare_pos)


def report_gate_pos_thread():
    rate = rospy.Rate(20)  # 20Hz

    while not rospy.is_shutdown():
        report_gate_pos()
        rate.sleep()

def report_flare_pos_thread():
    rate = rospy.Rate(20)  # 20Hz

    while not rospy.is_shutdown():
        report_flare_pos()
        rate.sleep()

def quaternion_to_euler(orientation):
    # Extract the quaternion components
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    roll = math.degrees(roll_x)
    pitch = math.degrees(pitch_y)
    yaw = math.degrees(yaw_z)

    return Vector3(roll, pitch, yaw)


def on_imu(imu: Imu):
    orientation = quaternion_to_euler(imu.orientation)

    actual_orientation_pub.publish(orientation)

    # add noise to the orientation
    noise_mean = 0.0  # Mean of the noise
    noise_std = 1.5  # Standard deviation of the noise

    orientation.x += np.random.normal(noise_mean, noise_std)
    orientation.y += np.random.normal(noise_mean, noise_std)
    orientation.z += np.random.normal(noise_mean, noise_std)

    orientation_pub.publish(orientation)


# subscribers
thrust_response = rospy.Subscriber("/rose_tvmc/thrust", Float32MultiArray, on_thrust)
depth_response = rospy.Subscriber("/emulation/pressure", FluidPressure, on_depth)
hull_front = rospy.Subscriber("/diagnostics/hull_front", Odometry, on_hull_front)
hull_back = rospy.Subscriber("/diagnostics/hull_back", Odometry, on_hull_back)
hull_back_right = rospy.Subscriber("/diagnostics/hull_back_right", Odometry, on_hull_back_right)
imu_sub = rospy.Subscriber("/diagnostics/imu", Imu, on_imu)


if __name__ == "__main__":
    rospy.init_node("sim_translation_layer")
    rospy.loginfo("Started Simulator Translation Layer")

    # Create a new thread
    gate_pos_thread = threading.Thread(target=report_gate_pos_thread, daemon=True)
    flare_pos_thread = threading.Thread(target=report_flare_pos_thread, daemon=True)

    # Start the thread
    gate_pos_thread.start()
    flare_pos_thread.start()

    rospy.spin()
