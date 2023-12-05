import sys
import tf
import numpy as np
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
import iaps_ur5e_sim.msg
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from iaps_ur5e_sim.msg import ap_movePoseResult
import time 


target_force = -10.0
# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
    "my_cartesian_force_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

force_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[3]
traj_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[2]


switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)

def switch_controller(target_controller):
    """Activates the desired controller and stops all others from the predefined list above"""
    
    other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

    other_controllers.remove(target_controller)

    srv = LoadControllerRequest()
    srv.name = target_controller
    load_srv(srv)

    srv = SwitchControllerRequest()
    srv.stop_controllers = other_controllers
    srv.start_controllers = [target_controller]
    srv.strictness = SwitchControllerRequest.BEST_EFFORT
    switch_srv(srv)

def force_feedback_callback(data):

    global reached_desired_force
    global z_force
    print('force_feedback_callback')

    rospy.loginfo("{}".format(data))

    z_force = data.wrench.force.z
    print(z_force)

    if z_force < target_force:
        reached_desired_force = True


def publish():
    print("Script launched")
    
    rospy.init_node("force_controller_test")
    rate = rospy.Rate(1) # frequency, 1 Hz

    frameBase = "base"
    frameTarget = "tool0_controller"
    tf_tcp = tf.TransformListener()
    print('Define publisher')
    force_controller_client = rospy.Publisher('/my_cartesian_force_controller/target_wrench', geometry_msgs.WrenchStamped, queue_size=10)
    
    
    print('Switching Controller')

    switch_controller(force_controller)


    force_goal = geometry_msgs.WrenchStamped()

    force_goal.wrench.force.x = 0.0
    force_goal.wrench.force.y = 0.0
    force_goal.wrench.force.z = target_force

    force_goal.wrench.torque.x = 0.0
    force_goal.wrench.torque.y = 0.0
    force_goal.wrench.torque.z = 0.0

    print('Publish')
    reached_desired_force = False
    while not reached_desired_force:
        force_controller_client.publish(force_goal)
        print('GOAL NOT REACH')
        time.sleep(1)
        feedback = rospy.wait_for_message("/wrench", geometry_msgs.WrenchStamped)
        

        if feedback.wrench.force.z < (target_force):       
            reached_desired_force = True
            print('Force reached')
            print(feedback)
            print(feedback.wrench.force.z)
        else:
            rate.sleep()


    print('switch back controller')
    switch_controller(traj_controller)


def subscribe_spin():
    force_feedback_subscriber = rospy.Subscriber('/wrench/wrench/force/z', geometry_msgs.WrenchStamped, force_feedback_callback)
    print('subscribe_spin')




if __name__ == '__main__':
    try:
        
        publish()
    except rospy.ROSInterruptException:
        print('switch back controller')
        switch_controller(traj_controller)