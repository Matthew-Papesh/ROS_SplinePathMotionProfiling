#!/usr/bin/env python3
from __future__ import annotations
from splines.srv import GetSimpleSplinePlan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import rospy
import handler
import math

class Navigation: 

    def __init__(self):
        """
        Initializes a Navigation node.
        """
        rospy.init_node("navigation", anonymous=True)

        self.node_rate = rospy.Rate(10)
        self.current_pose = PoseStamped()

        # publishers and subscribers
        self.cmd_vel_publisher = None
        self.cmd_vel_subscriber = None

        # subscriber flags:
        self.cmd_vel_subscriber_flag = False

        self.initPublishers()
        self.initSubscribers()
        rospy.sleep(1.0)
    
    def initPublishers(self):
        """
        Initializes and creates all node publishers.
        """
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def initSubscribers(self):
        """
        Initializes and creates all node subscribers. 
        """
        rospy.Subscriber("/odom", Odometry, self.handleLocalizationUpdate)
        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.handleCmdVelUpdate)

    def handleLocalizationUpdate(self, msg: Odometry):
        """
        Handles updating self.current_pose of the robot as the handling function designated 
        for the /odom subscriber.
        :param msg [Odometry] The specified data read from /odom topic
        """
        self.current_pose.pose.position.x = msg.pose.pose.position.x
        self.current_pose.pose.position.y = msg.pose.pose.position.y
        self.current_pose.pose.orientation = msg.pose.pose.orientation

    def handleCmdVelUpdate(self, msg: Twist):
        self.cmd_vel_subscriber_flag = True
        self.node_rate.sleep()

    def requestSimpleSplinePlan(self, start: PoseStamped, goal: PoseStamped) -> tuple[Path, list]:
        """
        Requests a interpolated spline path of poses given a specified starting and goal position. 
        :param start [PoseStamped] The specified start pose
        :param goal [PoseStamped] The specified goal pose
        :returns a simple spline Path object
        """
        rospy.loginfo("Navigation.py: Requesting simple spline path from \'/quintic_spline_path/simple_spline_plan\' service")
        rospy.wait_for_service("/quintic_spline_path/simple_spline_plan")
        try:
            client = rospy.ServiceProxy("/quintic_spline_path/simple_spline_plan", GetSimpleSplinePlan)
            simple_path = Path()
            simple_path.poses.append(start)
            simple_path.poses.append(goal)
            response = client(waypoints_path=simple_path)
            if response is None or response.spline_path is None:
                rospy.logerr("Navigation.py: error: failed to retrieve simple spline plan service response")
                exit()
            rospy.loginfo("Navigation.py: simple spline path service ended; returning path")
            return (response.spline_path, response.sd_steps) 
        except rospy.ServiceException as e:
            rospy.logerr("Navigation.py: exception thrown: service call failed => exception: " + e.__str__())
        return None

    def setSpeed(self, linear_speed: float, angular_speed: float):
        """
        Sets the speed of the robot with a given linear speed that is the tangential speed in [m/sec] with angular speed 
        in [radians/sec]
        :param linear_speed [float] The speficied tangential speed
        :param angular_speed [float] The specified angular speed
        """
        speed = Twist()
        speed.linear.x = linear_speed
        speed.angular.z = angular_speed

        while not self.cmd_vel_subscriber_flag:
            self.cmd_vel_publisher.publish(speed)
            self.node_rate.sleep()
        self.cmd_vel_subscriber_flag = False

    def rotateDrive(self, radians: float, angular_speed: float):
        """
        Rotates in-place a specified angle in radians such that a positive angle is counter-clockwise and negative is clockwise
        at a specified speed in [radians/sec]
        :param radians [float] The specified angle in radians
        :param angular_speed [float] The specified angular speed 
        """
        if radians == 0 or angular_speed == 0:
            return
        init_heading = handler.get_heading(self.current_pose)
        angular_speed = (abs(radians) / radians) * abs(angular_speed)

        self.setSpeed(0, angular_speed)
        while abs(handler.get_heading(self.current_pose) - init_heading) <= abs(radians):
            self.node_rate.sleep()
        self.setSpeed(0, 0)

    def forwardDrive(self, distance: float, linear_speed: float):
        """
        Drive a specified distance in a straight line such that a positive distance is a forward drive and negative a backward drive
        at a specified linear speed in [m/sec]
        :param distance [float] The specified distance to travel 
        :param linear_speed [float] The specified linear speed 
        """
        if distance == 0 or linear_speed == 0:
            return
        init_pose = PoseStamped()
        init_pose.pose.position.x = self.current_pose.pose.position.x
        init_pose.pose.position.y = self.current_pose.pose.position.y
        linear_speed = (abs(distance) / distance) * abs(linear_speed)
        
        self.setSpeed(linear_speed, 0)
        while handler.euclid_distance((init_pose.pose.position.x, init_pose.pose.position.y), (self.current_pose.pose.position.x, self.current_pose.pose.position.y)) <= distance:
            self.node_rate.sleep()
        self.setSpeed(0, 0)

    def splineDrive(self, spline_path: Path, spline_sd_steps: list, speed: float):
        
        def getPoseIndexByMSE(path: Path, pose: PoseStamped) -> int:
            min_loss = None
            ideal_pose_index = 0
            for i in range(0, len(path.poses)):
                path_pose = path.poses[i]
                x_loss = pow(path_pose.pose.position.x - pose.pose.position.x, 2.0)
                y_loss = pow(path_pose.pose.position.y - pose.pose.position.y, 2.0)
                loss = (x_loss + y_loss) / 2.0
                if min_loss is None or loss < min_loss:
                    min_loss = loss
                    ideal_pose_index = i
            return ideal_pose_index

        def getPoseSpeeds(spline_path: Path, sd_steps: list, index_0, index_f, speed_0, acceleration) -> tuple[list, list]:
            tolerance = 1 # how much reach +/- the index to select other points for approximating a spline circle
            base_index = tolerance
            acceleration = max(0.0001, abs(acceleration)) * (acceleration / max(0.0001, abs(acceleration))) 

            # computed speeds
            linear_speeds = []
            angular_speeds = []
            speed_init = False

            for index in range(index_0, index_f + 1):
                # scroll tolerance range while iterating
                if index > base_index and index <= len(spline_path.poses) - tolerance - 1:
                    base_index = index
                # compute instantaneous circle approximating continuous spline at given point
                p0, p1, p2 = spline_path.poses[base_index - tolerance], spline_path.poses[base_index], spline_path.poses[base_index + tolerance]
                (x_ICC, y_ICC, R) = handler.get_circle((p0.pose.position.x, p0.pose.position.y), (p1.pose.position.x, p1.pose.position.y), (p2.pose.position.x, p2.pose.position.y))
                v_0 = speed_0 if not speed_init else linear_speeds[len(linear_speeds) - 1]
                A = -v_0 / acceleration
                B = pow(abs(pow(v_0, 2.0) + 2.0*acceleration*sd_steps[index]), 0.5) / acceleration
                t_plus, t_minus = A + B, A - B
                delta_t = max(t_plus, t_minus)
                # compute sign of angular velocity
                linear_speeds.append(v_0 + delta_t*acceleration)
                delta_theta = handler.get_heading(p2) - handler.get_heading(p0)
                w_sgn = delta_theta / max(0.0001, abs(delta_theta))
                angular_speeds.append((abs(linear_speeds[len(linear_speeds) - 1]) / max(0.0001, abs(R))) * w_sgn)
                speed_init = True

                # t = [-v_0] / a (+/-) [sqrt(v_0^2 + 2a*s_d)] / a
                # => T = max(t_+, t_-)
                # v_f = v_0 + a*T       
            return (linear_speeds, angular_speeds)

        mid_point_index = int(len(spline_path.poses) - (len(spline_path.poses) / 2) - 1)
        accel_speeds = getPoseSpeeds(spline_path, spline_sd_steps, 0, mid_point_index, 0.0, 0.1)
        deccel_speeds = getPoseSpeeds(spline_path, spline_sd_steps, mid_point_index, len(spline_path.poses) - 1, accel_speeds[0][mid_point_index], -0.1)

        lin_speeds, ang_speeds = accel_speeds[0] + deccel_speeds[0], accel_speeds[1] + deccel_speeds[1]
        print("linear speeds:")
        print(lin_speeds)
        print("angular speeds:")
        print(ang_speeds)

        print("spline start: " + str(len(spline_path.poses)))
        index = 0
        #for index in range(0, len(spline_path.poses)):
        while index < len(spline_path.poses) - 1:
            index = getPoseIndexByMSE(spline_path, self.current_pose)
            self.setSpeed(abs(lin_speeds[index]), ang_speeds[index])
        # come to a stop
        self.setSpeed(0, 0)

    def run(self):
        # for i in range(0, 2):
        #     self.forwardDrive(1, 0.1)
        #     rospy.sleep(0.25)
        #     self.rotateDrive(math.pi, 0.1)
        #     rospy.sleep(0.25)

        goal = PoseStamped()
        goal.pose.position.x = 4
        goal.pose.position.y = 2
        goal.pose.orientation = handler.get_orientation(-math.pi / 4.0)
        
        spline_plan = self.requestSimpleSplinePlan(self.current_pose, goal)
        spline_path = spline_plan[0]
        spline_sd_steps = spline_plan[1]
        self.splineDrive(spline_path = spline_path, spline_sd_steps = spline_sd_steps, speed = 0.0)
        rospy.spin()

if __name__ == "__main__":
    Navigation().run()