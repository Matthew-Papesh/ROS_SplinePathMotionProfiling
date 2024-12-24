#!/usr/bin/env python3
from __future__ import annotations
from ROS_SplinePathMotionProfiling.srv import GetSimpleSplinePlan
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

    def requestSimpleSplinePlan(self, waypoints: list) -> tuple[Path, list, list]:
        """
        Requests a interpolated spline path of poses given specified waypoints. 
        :param waypoints [list] The specified PoseStamped list of poses to constraint a spline onto
        :returns a simple spline Path object
        """
        rospy.loginfo("Navigation.py: Requesting simple spline path from \'/quintic_spline_path/simple_spline_plan\' service")
        rospy.wait_for_service("/quintic_spline_path/simple_spline_plan")
        try:
            client = rospy.ServiceProxy("/quintic_spline_path/simple_spline_plan", GetSimpleSplinePlan)
            simple_path = Path()
            for waypoint in waypoints:
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.orientation = handler.get_orientation(waypoint[2])
                simple_path.poses.append(pose)

            response = client(waypoints_path=simple_path)
            if response is None or response.spline_path is None:
                rospy.logerr("Navigation.py: error: failed to retrieve simple spline plan service response")
                exit()
            rospy.loginfo("Navigation.py: simple spline path service ended; returning path")
            return (response.spline_path, response.sd_steps, response.cumulative_sd_steps) 
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

    def splineDrive(self, spline_path: Path, spline_sd_steps: list, spline_cumulative_sd_steps: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float):
        """
        Motion profiles and drives wheels speeds along a specified spline path given path poses, path arc distances, along with specified acceleration [m/sec^2], max linear speed [m/sec],
        max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]. 
        :param spline_path [Path] The specified spline path of interpolated poses
        :param spline_sd_steps [list] The specified list of float arc distances between a given spline path pose and the previous. 
        :param spline_cumulative_sd_steps [list] The specified list of float cumulative arc distances between a given spline path pose and the starting position. 
        :param acceleration [float] The specified acceleration in [m/sec^2] to profile by
        :param max_linear_speed [float] The specified max linear speed in [m/sec] to profile by
        :param max_angular_speed [float] The specified max angular speed in [radians/sec] to profile by
        :param max_centripetal_acceleration [float] The specified max centripetal acceleration in [m/sec^2] to profile by
        """

        def getPoseIndexByMSE(path: Path, pose: PoseStamped) -> int:
            """
            Calculates the pose index along a path the most closely approximates a specified pose. 
            Approximation is done by considering squared difference between path positions and the specified pose by MSE 
            to find the index with minimal error. 
            :param path [Path] The specified path
            :param pose [PoseStamped] The specified pose to consider
            :returns the most similar path position to that of the specified pose
            """
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

        def getPathSpeeds(spline_path: Path, sd_steps: list, cumulative_sd_steps: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float) -> tuple[list, list]:
            """
            Calculates speeds for each pose along a spline path. Linear and angular speeds are are determined given pose data while stepping through 
            the path to interpolate speeds knowing the specified acceleration. Pose data used come from the i-th pose along the spline, the i-th arc distance 
            between a pose and the previous on the spline from sd_steps, and the i-th arc distance between the initial position and the i-th pose from cumulative_sd_steps.
            Motion profiling criteria are taken into account as well. The following being the criteria of linear acceleration [m/sec^2], max linear speed [m/sec], 
            max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]; motion criteria clamps speeds within absolution bounderies between zero and 
            the maximums specified and only changes speeds by the rate of acceleration specified. Finally, speeds are return in the form of a tuple of linear and angular 
            speeds of the same length and corresponding order of the spline path specified. 

            :param spline_path [Path] The specified spline path to profile speeds for
            :param sd_steps [list] The specified list of float arc distances between a given pose and the previous on a spline path
            :param cumulative_sd_steps [list] The specified list of float cumulative arc distances between a given pose and the starting position of a spline path
            :param acceleration [float] The specified magnitude of acceleration in [m/sec^2] to profile by
            :param max_linear_speed [float] The specified max magnitude of tangential speed in [m/sec] to profile by
            :param max_angular_speed [float] The specified max magnitude of angular speed in [radians/sec] to profile by
            :param max_centripetal_acceleration [float] The specified max magnitude of centripetal acceleration in [m/sec^2] to profile by

            :returns a 2D tuple of linear speeds and angular speeds respectively in the same corresponding order as the spline path specified 
            """
            tolerance = 1 # how much reach +/- the index to select other points for approximating a spline circle
            base_index = tolerance 
            acceleration = max(0.00001, abs(acceleration)) 

            # computed speeds
            linear_speeds = []
            angular_speeds = []
            deccelerate = False
            # iterate through spline waypoints to compute speeds
            print("a_c's:")
            for index in range(0, len(spline_path.poses)):
                # scroll tolerance range while iterating
                if index > base_index and index <= len(spline_path.poses) - tolerance - 1:
                    base_index = index
                # compute instantaneous circle approximating continuous spline at given point
                p0, p1, p2 = spline_path.poses[base_index - tolerance], spline_path.poses[base_index], spline_path.poses[base_index + tolerance]
                (x_ICC, y_ICC, R) = handler.get_circle((p0.pose.position.x, p0.pose.position.y), (p1.pose.position.x, p1.pose.position.y), (p2.pose.position.x, p2.pose.position.y))
                
                # determine initial velocity (v_0) and arc distance (sd) coming from previous waypoint to the current
                v_0 = 0 if index == 0 else linear_speeds[len(linear_speeds) - 1]
                w_0 = 0 if index == 0 else angular_speeds[len(angular_speeds) - 1]
                sd = sd_steps[index]

                # determine the direction (sign of angular speed (w_sgn)) of which the robot will turn
                delta_theta = handler.get_heading(p2) - handler.get_heading(p0)
                w_sgn = delta_theta / handler.non_zero(abs(delta_theta), 0.00001)

                # compute current waypoint's linear and angular velocity by kinematics
                v_1 = pow(abs(pow(v_0, 2.0) + 2.0*sd*acceleration), 0.5)
                w_1 = v_1 / handler.non_zero(abs(R), 0.00001) * w_sgn
                # compute centripetal acceleration (a_c) and decceleration distance
                a_c = pow(v_1, 2.0) / handler.non_zero(abs(R), 0.00001)
                deccel_distance = abs(-pow(v_1, 2.0) / (2.0 * acceleration))

                # check to deccelerate or not: check if remaining spline distance is longer than the distance needed to deccelerate
                remaining_distance = abs(cumulative_sd_steps[len(cumulative_sd_steps) - 1] - cumulative_sd_steps[index])
                remaining_distance = remaining_distance if remaining_distance > 0.05 else 0

                if not deccelerate and remaining_distance <= deccel_distance:
                    # if just flagged True, the robot must of just noticed its speed, or any faster, would require slowing now to not overshoot the
                    # end of the spline. Once deccelerating, the robot should not wait to slow or ever go faster; don't consider handling those cases.
                    deccelerate = True
                    acceleration = -abs(acceleration)
                if abs(a_c) > abs(max_centripetal_acceleration):
                    # the robot must of just met/passed the max centripetal acceleration threshold; do not accelerate here but hold constant speed at the max linear speed.
                    w_1 = v_0 / handler.non_zero(abs(R), 0.00001) * w_sgn
                    v_1 = v_0
                elif abs(w_1) > abs(max_angular_speed):
                    # the robot must of just met/passed the max angular speed threshold; do not accelerate here but hold constant speed at the max angular speed.
                    w_1 = max_angular_speed * w_sgn
                    v_1 = max_angular_speed * R
                elif abs(v_1) > abs(max_linear_speed):
                    # the robot must of just met/passed the max linear speed threshold; do not accelerate here but hold constant speed at the max linear speed. 
                    w_1 = max_linear_speed / handler.non_zero(abs(R), 0.00001) * w_sgn
                    v_1 = max_linear_speed
                elif deccelerate and remaining_distance > deccel_distance:
                    # if the robot is slowing down to soon stop but find the stop may come too early, then hold speeds constants
                    w_1 = v_0 / handler.non_zero(abs(R), 0.00001) * w_sgn
                    v_1 = v_0

                print(format(pow(v_1, 2.0) / handler.non_zero(abs(R), 0.00001), '.6'))
                # add speeds
                linear_speeds.append(v_1)
                angular_speeds.append(w_1)
      
            return (linear_speeds, angular_speeds)

        # compute wheel speeds
        speeds = getPathSpeeds(spline_path, spline_sd_steps, spline_cumulative_sd_steps, acceleration, max_linear_speed, max_angular_speed, max_centripetal_acceleration)
        # print data
        lin_speeds, ang_speeds = speeds[0], speeds[1] 
        print("linear speeds:")
        print(lin_speeds)
        print("angular speeds:")
        print(ang_speeds)
        print("cumulative sd's:")
        print(spline_cumulative_sd_steps)
        print("spline start: " + str(len(spline_path.poses)))
        
        # drive robot with speed data
        index = 0
        while index < len(spline_path.poses) - 1:
            index = getPoseIndexByMSE(spline_path, self.current_pose)
            self.setSpeed(abs(lin_speeds[index]), ang_speeds[index])
        # come to a stop
        self.setSpeed(0, 0)

    def driveSplinePath(self, waypoints: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float):
        """
        Drives a path of splines given waypoints to constrain the spline onto with motion profiling constraints of acceleration [m/sec^2], 
        max linear speed [m/sec], max angular speed [radians/sec], and max centripetal acceleration [m/sec^2].
        :param waypoints [list] The specified list of pose vectors for each waypoint of which the spline path should intersect through
        :param acceleration [float] The specified acceleration in [m/sec^2] to profile by
        :param max_linear_speed [float] The specified max linear speed in [m/sec] to profile by
        :param max_angular_speed [float] The specified max angular speed in [radians/sec] to profile by
        :param max_centripetal_acceleration [float] The specified max centripetal acceleration in [m/sec^2] to profile by
        """
        # add initial robot position to the front of the waypoints and request interpolated spline path. 
        waypoints = [(self.current_pose.pose.position.x, self.current_pose.pose.position.y, handler.get_heading(self.current_pose))] + waypoints
        spline_plan = self.requestSimpleSplinePlan(waypoints)
        # drive spline given computed spline path and motion profiling constraints. 
        self.splineDrive(spline_plan[0], spline_plan[1], spline_plan[2], acceleration, max_linear_speed, max_angular_speed, max_centripetal_acceleration)

    def run(self):
        # absolute maximum centripetal acceleration given simulation physics
        coeff_static_friction = 1.0
        centripetal_acceleration = coeff_static_friction * 9.81 # [m/sec^2]
        # percentage of centripetal acceleration to consider when specifying max centripetal acceleration for spline path driving
        scaler = 0.3
        
        # other motion profiling constraints:
        acceleration = 0.1 # [m/sec^2]
        max_angular_speed = 1.0 # [radians/sec]
        max_linear_speed = 0.25 # [m/sec]
        
        # waypoints to travel through along spline path: (waypoint = (x, y, radians))
        waypoints = [(4,2,-math.pi/4.0), (5,1,-math.pi/2.0), (4, 0, -math.pi*3.0/4.0), (0, 0, math.pi)]

        # drive spline and spin ROS node
        self.driveSplinePath(waypoints, acceleration, max_angular_speed, max_linear_speed, centripetal_acceleration * scaler)
        rospy.spin()

if __name__ == "__main__":
    Navigation().run()
