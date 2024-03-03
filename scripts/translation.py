#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import FluidPressure,Imu
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Float64
from geometry_msgs.msg import Vector3, Pose 
from nav_msgs.msg import Odometry
import threading
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import math

StartPos = Vector3()

GATE_POS = (5, 2, 2.48)

# publishers
anglePub= rospy.Publisher("/emulation/orientation", Vector3, queue_size=1)
diagAnglePub = rospy.Publisher("/diagnostics/orientation", Vector3, queue_size=1)


depth_pub = rospy.Publisher("/depth_data", Float64, queue_size=1)
depth_data = Float64()

thrust_control = rospy.Publisher("/control/thrusters", Float64MultiArray, queue_size=1)
thrust_control_data = Float64MultiArray()

gate_localization_pub = rospy.Publisher("/localization/gate", Vector3, queue_size=1)

state = {
    'hull_front_pose': None,
    'hull_back_pose': None
}


def on_thrust(x: Float32MultiArray):
    thrust_control_data.data = [a / 100 for a in x.data]
    thrust_control.publish(thrust_control_data)


def on_depth(x: FluidPressure):
    depth_data.data = x.fluid_pressure / (9.8 * 997)
    depth_pub.publish(depth_data)


def on_hull_front(x: Odometry):
    state['hull_front_pose'] = x.pose.pose
    
    
def on_hull_back(x: Odometry):
    state['hull_back_pose'] = x.pose.pose
    

def calculate_hull_vector() -> np.ndarray:
    hull_front_pose = state['hull_front_pose']
    hull_back_pose = state['hull_back_pose']
    
    if hull_front_pose is None or hull_back_pose is None:
        return None
    
    front_position = np.array([hull_front_pose.position.x, hull_front_pose.position.y, hull_front_pose.position.z])
    back_position = np.array([hull_back_pose.position.x, hull_back_pose.position.y, hull_back_pose.position.z])
    
    hull_vector = front_position - back_position
    return hull_vector


def calculate_gate_vector() -> np.ndarray:
    hull_front_pose = state['hull_front_pose']
    
    if hull_front_pose is None:
        return None
    
    gate_position = np.array(GATE_POS)
    front_position = np.array([hull_front_pose.position.x, hull_front_pose.position.y, hull_front_pose.position.z])
    
    gate_vector = gate_position - front_position
    return gate_vector


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


def report_gate_pos():
    gate_vector = calculate_gate_vector()
    gate_angle_deg = gate_angle()

    print("Gate", gate_vector)
    print("Angle", gate_angle_deg)
    
    if gate_vector is not None and gate_angle_deg <= 40 and 0.5 < np.linalg.norm(gate_vector) < 7.5:
        gate_pos = Vector3()
        
        noise_mean = 0.0  # Mean of the noise
        noise_std = 0.1  # Standard deviation of the noise

        gate_pos.x = gate_vector[0] + np.random.normal(noise_mean, noise_std)
        gate_pos.y = gate_vector[1] + np.random.normal(noise_mean, noise_std)
        gate_pos.z = gate_vector[2] + np.random.normal(noise_mean, noise_std)
        
        gate_localization_pub.publish(gate_pos)


def report_gate_pos_thread():
    rate = rospy.Rate(20)  # 20Hz

    while not rospy.is_shutdown():
        report_gate_pos()
        rate.sleep()

def convertBecauseRoshanIsLazy(shit):
    angles = Vector3()
    # Import the necessary libraries
    # Function to convert quaternion to Euler angles
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

    # Subscriber callback function for IMU data

    # Unpack the quaternion data from the IMU message
    quaternion = shit.orientation

    # Convert quaternion to Euler angles
    angles = quaternion_to_euler(quaternion)
    angles_msg = Vector3()
    angles_msg.x = angles.x
    angles_msg.y = angles.y
    angles_msg.z = angles.z

    anglePub.publish(angles_msg)

def convertBecauseRoshanIsLazyDiag(shit):
    angles = Vector3()
    # Import the necessary libraries
    # Function to convert quaternion to Euler angles
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

    # Subscriber callback function for IMU data

    # Unpack the quaternion data from the IMU message
    quaternion = shit.orientation

    # Convert quaternion to Euler angles
    angles = quaternion_to_euler(quaternion)
    angles_msg = Vector3()
    angles_msg.x = angles.x
    angles_msg.y = angles.y
    angles_msg.z = angles.z

    diagAnglePub.publish(angles_msg)



# subscribers
thrust_response = rospy.Subscriber("/rose_tvmc/thrust", Float32MultiArray, on_thrust)
depth_response = rospy.Subscriber("/emulation/pressure", FluidPressure, on_depth)
hull_front = rospy.Subscriber("/diagnostics/hull_front", Odometry, on_hull_front)
hull_back = rospy.Subscriber("/diagnostics/hull_back", Odometry, on_hull_back)
angSub = rospy.Subscriber("/emulation/imu", Imu,convertBecauseRoshanIsLazy)
diagAngSub = rospy.Subscriber("/diagnostics/imu", Imu,convertBecauseRoshanIsLazyDiag)


if __name__ == "__main__":
    rospy.init_node("sim_translation_layer")
    rospy.loginfo("Started Simulator Translation Layer")

    # Create a new thread
    gate_pos_thread = threading.Thread(target=report_gate_pos_thread, daemon=True)

    # Start the thread
    gate_pos_thread.start()

    rospy.spin()
