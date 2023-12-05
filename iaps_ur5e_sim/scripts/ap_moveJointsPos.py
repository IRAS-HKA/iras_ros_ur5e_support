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

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

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
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class AP_MOVE_JOINTSPOS(object):
    # create messages that are used to publish feedback/result
    _feedback = geometry_msgs.Pose()
    _result = std_msgs.Bool()
    timeout = rospy.Duration(5)

    def __init__(self, name):
        
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, iaps_ur5e_sim.msg.ap_moveJointsPosAction, execute_cb=self.execute_cb, auto_start = False)

        # set trajectory options
        self.traj_time = 10.0       # total trajectory execution time in sec

        # select desired cartesian controller
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[2]

        # get controller client
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
        rospy.loginfo("{}: connecting to controller {}".format(self._action_name, self.joint_trajectory_controller))
        # Wait for action server to be ready
        if not self.trajectory_client.wait_for_server(self.timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)
        rospy.loginfo("{}: connected".format(self._action_name))

        # get services to switch and load desired controller
        self.switch_srv = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        try:
            self.switch_srv.wait_for_service(self.timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self._as.start()

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)
    
    def execute_cb(self, goal_jointspos):
        """Creates a joint trajectory and sends it using the selected action server"""

        # load and switch to desired controller, stop other controllers
        self.switch_controller(self.joint_trajectory_controller)

        # define goal message
        #goal_jointspos.point.time_from_start = rospy.Duration(self.traj_time)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points.append(goal_jointspos.point)

        # send trajectory to controller and wait for result
        rospy.loginfo("{}: executing joint trajectory".format(self._action_name))

        self.trajectory_client.send_goal(goal)

        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()

        rospy.loginfo("{}: Trajectory execution finished in state {}".format(self._action_name, result.error_code))

        self._result = True
        self._as.set_succeeded(self._result)
           
        
if __name__ == '__main__':
    rospy.init_node('ap_moveJointsPos')
    server = AP_MOVE_JOINTSPOS(rospy.get_name())
    rospy.spin()
