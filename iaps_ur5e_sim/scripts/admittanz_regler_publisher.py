import sys
import rospy
import actionlib
import tf
import numpy as np
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs


class admittanz_regler(object):

    def __init__(self, name):
        self._action_name = name
        
        # freiheitsgrade aktivieren(1)/deaktivieren(0)
        # [x, y, z, u, v, w], y is inverted
        self.activation_mask = np.array([0.0, 0.0, 1.0, 0, 0, 0]) 

        # Reglerparameter [x, y, z, u, v, w]
        self.M = np.array([200.0, 200.0, 100.0, 200.0, 200.0, 200.0])
        self.C = np.array([50.0, 50.0, 80.0, 50.0, 50.0, 50.0])   #  38744.0
        #self.D = np.array([1.0, 1.0, 500.0, 1.0, 1.0, 1.0])
        self.D = 2.828427 * 2 * np.sqrt(self.M * self.C)
    
        rospy.loginfo('M: %s' % str(self.M))
        rospy.loginfo('D: %s' % str(self.D))
        rospy.loginfo('C: %s' % str(self.C))
                
        # Geschwindigkeitslimits
        self.vel_limit_linear = 0.02    # m/s, higher value gripper approach too fast if current force=0
        self.vel_limit_angular = 0.02   # rad/s     
        
        # sampling time
        self.Hz = 50
        self.T = 1/self.Hz
        rospy.loginfo('Hz: %s' % str(self.Hz))

        # initialisiere twist
        self.xddd = np.zeros(6)
        self.xdd = np.zeros(6)
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

        self.goal_client = rospy.Subscriber("/admittanz_regler/desired_wrench", geometry_msgs.WrenchStamped, self.goal_cb)

        # start control loop
        self.control_loop()

    def control_loop(self):
        rate = rospy.Rate(self.Hz)

        while not rospy.is_shutdown():
            
            error = self.target_wrench - self.actual_wrench

            self.error_wrench = self.error_threshold(error)

            # calculate twist via admittance rule
            self.calculated_twist = (1/self.C) * self.error_wrench

            #self.xddd = (1/self.M) * (self.error_wrench - self.D * self.xdd - self.C * self.calculated_twist)
            #self.xdd = self.xdd + self.xddd * self.T
            #self.calculated_twist = self.calculated_twist + self.xdd * self.T
            #rospy.loginfo('xddd: %s' % str(self.xddd))

            # activate / deactivate axes
            self.target_twist = self.select_axes(self.calculated_twist)

            # clip twist velocity to limits
            #self.target_twist = self.clip_twist(self.target_twist)

            # transform twist vector to ros twist msg
            self.target_twist = self.vec2twist(self.target_twist)

            # send twist msg to controller    
            self.controller_client.publish(self.target_twist)

            rate.sleep()

    def goal_cb(self, goal_wrench):
        # transform ros wrench msg to vector
        self.target_wrench = self.wrench2vec(goal_wrench.wrench)

    def wrench_cb(self, data):
        # transform ros wrench msg to vector
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

    def error_threshold(self, error):
        mask = np.abs(error) > 0.01
        return error * mask


if __name__ == '__main__':
    rospy.init_node('admittanz_regler')
    mycontroller = admittanz_regler(rospy.get_name())
    rospy.spin()
