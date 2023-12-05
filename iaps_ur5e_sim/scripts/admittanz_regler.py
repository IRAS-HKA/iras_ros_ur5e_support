import sys
import rospy
import actionlib
import tf
import numpy as np
from iaps_ur5e_sim.msg import ap_movePoseActionGoal, ap_movePoseActionResult
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
import rosbag


class admittanz_regler(object):
    _feedback = geometry_msgs.Twist()
    _result = std_msgs.Bool()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 
                                                iaps_ur5e_sim.msg.admittanz_reglerAction, 
                                                execute_cb = self.execute_cb, 
                                                auto_start = False)
        
        # freiheitsgrade aktivieren(1)/deaktivieren(0)
        # [x, y, z, u, v, w], y is inverted
        self.activation_mask = np.array([1.0, -1.0, 1.0, 0, 0, 0]) 

        # Reglerparameter [x, y, z, u, v, w]
        self.M = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0])
        self.D = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0])
        self.C = np.array([40.0, 40.0, 40.0, 1.0, 1.0, 1.0])   #  38744.0
        rospy.loginfo('C: %s' % str(self.C[2]))
                
        # Geschwindigkeitslimits
        self.vel_limit_linear = 0.02    # m/s, higher value gripper approach too fast if current force=0
        self.vel_limit_angular = 0.02   # rad/s     
        
        # sampling time
        self.Hz = 50
        self.T = 1/self.Hz
        rospy.loginfo('Hz: %s' % str(self.Hz))

        # initialisiere twist
        self.calculated_twist = np.zeros(6)            # calculated twist from admittance control

        # initialisiere wrench
        self.target_wrench = np.zeros(6)
        self.actual_wrench = np.zeros(6)

        # subscribe to ft-sensor (UR5e sensor: /wrench, Schunk sensor: /ftn_axia)
        self.wrench_client = rospy.Subscriber("/ftn_axia", geometry_msgs.WrenchStamped, self.wrench_cb)

        # controller action topic
        self.controller_client = rospy.Publisher('/twist_controller/command', geometry_msgs.Twist, queue_size=10)

        # service to switch/load controllers
        self.switch_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        self.load_srv = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)

        # list of available controllers
        self.JOINT_TRAJECTORY_CONTROLLERS = [
            "scaled_pos_joint_traj_controller",
            "scaled_vel_joint_traj_controller",
            "pos_joint_traj_controller",
            "vel_joint_traj_controller",
            "forward_joint_traj_controller",
        ]

        # All of those controllers can be used to execute Cartesian trajectories.
        # The scaled versions should be preferred over the non-scaled versions.
        self.CARTESIAN_TRAJECTORY_CONTROLLERS = [
            "pose_based_cartesian_traj_controller",
            "joint_based_cartesian_traj_controller",
            "forward_cartesian_traj_controller",
            "my_cartesian_force_controller",
        ]

        # We'll have to make sure that none of these controllers are running, as they will
        # be conflicting with the joint trajectory controllers
        self.CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

        self.switch_controller(target_controller="twist_controller")
        
        # set base and target frame
        self.frameBase = "base"
        self.frameTarget = "tool0_controller" 

        self._as.start()     

    def execute_cb(self, goal_wrench):
        rate = rospy.Rate(self.Hz)
        success = True

        while not rospy.is_shutdown():
    
            if self._as.is_preempt_requested():
                twist = geometry_msgs.Twist()   # twist with every velocity zero
                self.controller_client.publish(twist)
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            
            # transform ros wrench msg to vector
            self.target_wrench = self.wrench2vec(goal_wrench.wrench)
            self.error_wrench = self.error_threshold()

            # calculate twist via admittance rule
            #self.calculated_twist = (1/self.C) * (self.target_wrench - self.actual_wrench)
            self.calculated_twist = (1/self.C) * self.error_wrench

            # activate / deactivate axes
            self.calculated_twist = self.select_axes(self.calculated_twist)

            # clip twist velocity to limits
            #self.calculated_twist = self.clip_twist(self.calculated_twist)

            # transform twist vector to ros twist msg
            self.target_twist = self.vec2twist(self.calculated_twist)

            # send twist msg to controller    
            self.controller_client.publish(self.target_twist)

            rate.sleep()
        
        if success:
            twist = geometry_msgs.Twist()   # twist with every velocity zero
            self.controller_client.publish(twist)
            self._result = True
            self._as.set_succeeded(self._result)
              
    def wrench_cb(self, data):
        self.actual_wrench = self.wrench2vec(data.wrench)

    def wrench2vec(self, wrench):
        force = wrench.force
        torque = wrench.torque

        return np.array([force.x, force.y, force.z, torque.x, torque.y, torque.z])

    def twist2vec(self, twist):
        linear = twist.linear
        angular = twist.angular

        return np.array([linear.x, linear.y, linear.z, angular.x, angular.y, angular.z])

    def vec2twist(self, vec):
        twist = geometry_msgs.Twist()

        twist.linear.x = vec[0]
        twist.linear.y = vec[1]
        twist.linear.z = vec[2]

        twist.angular.x = vec[3]
        twist.angular.y = vec[4]
        twist.angular.z = vec[5]

        return twist
    
    def clip_twist(self, twist):
        twist[0:3] = np.clip(twist[0:3], -self.vel_limit_linear, self.vel_limit_linear)
        twist[3:6] = np.clip(twist[3:6], -self.vel_limit_angular, self.vel_limit_angular)

        return twist   

    def switch_controller(self, target_controller):
        """
        Activates the desired controller and stops all others from the predefined list in __init__
        """
        
        other_controllers = (
                self.JOINT_TRAJECTORY_CONTROLLERS
                + self.CARTESIAN_TRAJECTORY_CONTROLLERS
                + self.CONFLICTING_CONTROLLERS
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

    def select_axes(self, twist):
        return self.activation_mask * twist

    def error_threshold(self):
        error = self.target_wrench - self.actual_wrench
        mask = np.abs(error) > 0.8
        return error * mask


if __name__ == '__main__':
    rospy.init_node('admittanz_regler')
    server = admittanz_regler(rospy.get_name())
    rospy.spin()
