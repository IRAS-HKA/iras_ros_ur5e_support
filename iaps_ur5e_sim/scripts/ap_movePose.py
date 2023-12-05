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


class AP_MOVE_POSE(object):
    # create messages that are used to publish feedback/result
    _feedback = geometry_msgs.Pose()
    _result = ap_movePoseResult()
    timeout = rospy.Duration(5)



    def __init__(self, name):
        
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, iaps_ur5e_sim.msg.ap_movePoseAction, execute_cb=self.execute_cb, auto_start = False)
        
        # set base and target frame
        self.frameBase = "base"
        self.frameTarget = "tool0_controller"  # real: "tool0_controller", sim: "tool0"

        # set trajectory options
        self.steps = 10             # how many steps used to interpolate position and orientation
        self.traj_time = 5 #0.1     # total trajectory execution time in sec # 6

        # transform listener to get current tcp pose
        self.tf_tcp = tf.TransformListener()

        # select desired cartesian controller
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[2]

        # get controller client
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )
        rospy.loginfo("{}: connecting to controller {}".format(self._action_name, self.cartesian_trajectory_controller))
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

        self.switch_controller(self.cartesian_trajectory_controller)
        
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
    
    def interp_orientation(self, q1, q2, steps):
        waypoints = []
        steps = steps - 1
        for k in range(steps):
            q_k = tf.transformations.quaternion_slerp(q1, q2, k/steps)
            waypoints.append(q_k)
        waypoints.append(q2)

        return waypoints

    def interp_position(self, tvec1, tvec2, steps):
        waypoints = []
        steps = steps - 1
        for k in range(steps):
            tvec_k = ((steps-k)/steps) * tvec1 + k/steps * tvec2
            waypoints.append(tvec_k)
        waypoints.append(tvec2)

        return waypoints

    def execute_cb(self, goal_pose):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        g_pos = goal_pose.pose.position #+ current_T[0]
        g_rot = goal_pose.pose.orientation

        # load and switch to desired controller, stop other controllers
        self.switch_controller(self.cartesian_trajectory_controller) # Achtung normal einkommentiert ####################################

        # define goal message
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory.controlled_frame = self.frameTarget
        goal.trajectory.header.frame_id = self.frameBase
        # testing for forwarding
        point = CartesianTrajectoryPoint()
        point.pose = goal_pose.pose
        point.time_from_start = rospy.Duration(self.traj_time)
        goal.trajectory.points.append(point)

        # get current tcp pose
        #(trans, rot) = self.tf_tcp.lookupTransform(self.frameBase, self.frameTarget, rospy.Time(0))

        # start pose is current pose
        #tv1 = np.array(trans)
        #q1 = rot

        # target/goal pose
        tv2 = np.array([g_pos.x, g_pos.y, g_pos.z])
        q2 = np.array([g_rot.x, g_rot.y, g_rot.z, g_rot.w]) # rot #

        # calculate cartesian waypoints by linear interpolation
        #wp = self.interp_orientation(q1, q2, self.steps)
        #wp_t = self.interp_position(tv1, tv2, self.steps)
        rospy.loginfo("{}: recieved goal position: {}, orientation: {}".format(self._action_name, tv2, q2))

        # make list of poses from waypoints
        #pose_list = []
        #for i in range(self.steps):
        #    pose_k = geometry_msgs.Pose(geometry_msgs.Vector3(wp_t[i][0], wp_t[i][1], wp_t[i][2]), 
        #                                geometry_msgs.Quaternion(wp[i][0], wp[i][1], wp[i][2], wp[i][3]))
        #    pose_list.append(pose_k)

        # define time for each waypoint
        #duration_list = self.traj_time/self.steps * np.arange(1, self.steps+1, 1)

        # write everything in one TrajectoryGoal message
        #for i, pose in enumerate(pose_list):
        #    point = CartesianTrajectoryPoint()
        #    point.pose = pose
        #    point.time_from_start = rospy.Duration(duration_list[i])
        #    point.twist.linear.z = float("NaN")         # no stopping at waypoints, smooth motion
        #    goal.trajectory.points.append(point)

        # send trajectory to controller and wait for result
        rospy.loginfo("{}: executing cartesian trajectory".format(self._action_name))

        self.trajectory_client.send_goal(goal)

        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()

        rospy.loginfo("{}: Trajectory execution finished in state {}".format(self._action_name, result.error_code))

        self._result.success = True
        self._as.set_succeeded(self._result)
           
        
if __name__ == '__main__':
    rospy.init_node('ap_movePose')
    server = AP_MOVE_POSE(rospy.get_name())
    rospy.spin()
