#!/usr/bin/env python3
from __future__ import annotations
from ROS_SplinePathMotionProfiling.srv import GetSimpleSplinePlan 
from ROS_SplinePathMotionProfiling.srv import GetNavCriteriaPlan, GetNavCriteriaPlanResponse
from ROS_SplinePathMotionProfiling.srv import GetNavSimTest, GetNavSimTestResponse
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path, GridCells
from std_srvs.srv import Empty
from PID import PID
import rospy
import handler
import math
import heapq

# testing flags:
INTERNAL_TESTING = False

class Navigation: 

    def __init__(self):
        """
        Initializes a Navigation node.
        """
        rospy.init_node("navigation", anonymous=True)
        self.node_rate = rospy.Rate(10)
        self.current_pose = PoseStamped()
        self.current_speed = Twist()

        # publishers and subscribers
        self.spline_gridcells_publisher = None
        self.driving_gridcells_publisher = None
        self.driving_gridcells_subscriber = None
        self.cmd_vel_publisher = None
        self.cmd_vel_subscriber = None

        # subscriber flags:
        self.cmd_vel_subscriber_flag = False

        # motion profiling criteria:
        self.ACCELERATION = 0.0 # [m/sec^2]
        self.MAX_LINEAR_SPEED = 0.0 # [m/sec]
        self.MAX_ANGULAR_SPEED = 0.0 # [m/sec]
        self.MAX_CENTRIPETAL_ACCELERATION = 0.0 # [m/sec^2]
        # other robot constraints: 
        self.MAX_SPLINE_TURN = 0.5 * math.pi # [radians]; the max amount the robot should turn when driving a spline
        self.TURTLEBOT3_RADIUS = 0.105 # [m]


        # pid feedback coefficients (linear and angular differential speed PID)
        self.ANG_KP, self.ANG_KI, self.ANG_KD = 4.0, 0.0001, 4.0
        self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.0, 0.001, 0.5 
        # PCT TEST #1
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.265625, 0.0000835, 14.829688
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656  
        # Found Coefficients: 
        # Angular: kp = 15.152344, ki = 0.000083, kd = 15.357032
        # Linear: kp = 1.050781, ki = 0.002802, kd = 1.097656

        # PCT TEST #2
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.152344, 0.000083, 15.357032
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656
        # Found Coefficients: 
        # Angular: kp = 14.990235, ki = 0.000083, kd = 15.184766
        # Linear: kp = 1.050781, ki = 0.002802, kd = 1.097656

        # PCT TEST #3
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 14.990235, 0.000083, 15.184766
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656
        # Found Coefficients: 
        # Angular: kp = 15.097657, ki = 0.000083, kd = 14.949805
        # Linear: kp = 1.050781, ki = 0.002802, kd = 1.097656

        self.initPublishers()
        self.initSubscribers()
        rospy.sleep(1.0)

    def initPublishers(self):
        """
        Initializes and creates all node publishers.
        """
        self.spline_gridcells_publisher = rospy.Publisher("/navigation/spline_gridcells", GridCells, queue_size=10)
        self.driving_gridcells_publisher = rospy.Publisher("/navigation/recorded_driving_gridcells", GridCells, queue_size=10)
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
        self.current_speed = msg

    def resetRobot(self):
        """
        Requests a reset of the robot in Gazebo World Simulation.
        """
        rospy.loginfo("Navigation.py: Requesting Robot Reset from \'/gazebo/set_model_state\' service")
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            state_msg = ModelState()
            state_msg.model_name = "turtlebot3_burger"
            state_msg.reference_frame = "map"
            state_msg.pose.position.x = 0
            state_msg.pose.position.y = 0
            state_msg.pose.orientation = handler.get_orientation(0)
            response = client(state_msg)
            if response is None:
                rospy.logerr("Navigation.py: error: failed to reset robot on service response")
                exit()
            rospy.loginfo("Navigation.py: robot reset service ended successfuly")
        except rospy.ServiceException as e:
            rospy.logerr("Navigation.py: exception thrown: service call failed => exception: " + e.__str__())

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
            try:
                self.node_rate.sleep()
            except:
                pass
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
            try:
                self.node_rate.sleep()
            except:
                pass
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
            try:
                self.node_rate.sleep()
            except:
                pass
        self.setSpeed(0, 0)

    def rvizViewSplinePathProgression(self, spline_path: Path, spline_index: int, padding: float): 
        """
        Visualizes path progression in RViz by publishing a local range of path points in world frame of reference to be viewed
        as GridCells. The local range around where the program thinks the robot is along a path is considered so that to limit the amount of
        points considered when approximating the next point. 
        :param spline_path [Path] The specified path
        :param spline_index [int] The specified base point as an index on a path to consider
        :param padding [int] The specified radius or padding from the base spline index point to consider as the local range or points
        """
        # visualize padded local area considered for approximation; visualize by gridcells
        padding_x, padding_y = [], []
        for kernel_i in range(-padding, padding + 1):
            try:
                # exception handling for out-of-bounds errors
                padding_x.append(spline_path.poses[kernel_i + spline_index].pose.position.x)
                padding_y.append(spline_path.poses[kernel_i + spline_index].pose.position.y)
            except:
                continue
        gridcells = handler.get_gridcells_by_list((0,0), 1.0, padding_x, padding_y)
        self.spline_gridcells_publisher.publish(gridcells)

    def rvizViewDrivingProgression(self, recorded_path: Path):
        """
        Visualizes driving progression of the robot as a path of recorded poses such that each pose is read from robot localization as the closest 
        the robot came to driving along a given waypoint along a spline path of which it was currently following; displays the closest the robot came 
        to each waypoint along a spline path. 
        """
        gridcells = handler.get_gridcells_by_path((0,0), 1.0, recorded_path)
        self.driving_gridcells_publisher.publish(gridcells)

    def getPoseIndex(self, spline_path: Path, pose: PoseStamped, kernel_index: int, padding: int) -> tuple[int]:
        """
        Calculates the pose index along a path that most closely approximates a specified pose. 
        :param path [Path] The specified path
        :param pose [PoseStamped] The specified pose to consider
        :param kernel_index [int] The specified position of the kernel or scope to evaluate 
        :param padding [int] The specified radius of the kernel or scope to evaluate
        :returns the most similar path position to that of the specified pose as an index
        """
        min_loss = None
        ideal_pose_index = 0 
        for i in range(max(0, kernel_index - padding), min(kernel_index + padding + 1, len(spline_path.poses))):
            path_pose = spline_path.poses[i]
            x_loss = abs(path_pose.pose.position.x - pose.pose.position.x)
            y_loss = abs(path_pose.pose.position.y - pose.pose.position.y)
            loss = (x_loss + y_loss) / 2.0
            if min_loss is None or loss < min_loss:
                min_loss = loss
                ideal_pose_index = i
        return ideal_pose_index

    def getPathSpeeds(self, spline_path: Path, sd_steps: list, cumulative_sd_steps: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float) -> tuple[list, list]:
        """
        Calculates speeds for each pose along a spline path. Linear and angular speeds are are determined given pose data while stepping through 
        the path to interpolate speeds knowing the specified acceleration. Pose data used come from the i-th pose along the spline, the i-th arc distance 
        between a pose and the previous on the spline from sd_steps, and the i-th arc distance between the initial position and the i-th pose from cumulative_sd_steps.
        Motion profiling criteria are taken into account as well. The following being the criteria of linear acceleration [m/sec^2], max linear speed [m/sec], 
        max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]; motion criteria clamps speeds within absolution boundaries between zero and 
        the maximums specified and only changes speeds by the rate of acceleration specified. Finally, speeds are return in the form of a tuple of linear and angular 
        speeds of the same length and corresponding MSE order of the spline path specified. 

        :param spline_path [Path] The specified spline path to profile speeds for
        :param sd_steps [list] The specified list of float arc distances between a given pose and the previous on a spline path
        :param cumulative_sd_steps [list] The specified list of float cumulative arc distances between a given pose and the starting position of a spline path
        :param acceleration [float] The specified magnitude of acceleration in [m/sec^2] to profile by
        :param max_linear_speed [float] The specified max magnitude of tangential speed in [m/sec] to profile by
        :param max_angular_speed [float] The specified max magnitude of angular speed in [radians/sec] to profile by
        :param max_centripetal_acceleration [float] The specified max magnitude of centripetal acceleration in [m/sec^2] to profile by
        :returns a tuple of linear speeds and angular speeds respectively in the same corresponding order as the spline path specified 
        """
        tolerance = 1 # how much reach +/- the index to select other points for approximating a spline circle
        base_index = tolerance 
        acceleration = max(0.00001, abs(acceleration)) 
        # computed speeds
        linear_speeds = []
        angular_speeds = []
        deccelerate = False
        # iterate through spline waypoints to compute speeds
        for index in range(0, len(spline_path.poses)):
            # scroll tolerance range while iterating
            if index > base_index and index <= len(spline_path.poses) - tolerance - 1:
                base_index = index
            # compute instantaneous circle approximating continuous spline at given point
            p0, p1, p2 = spline_path.poses[base_index - tolerance], spline_path.poses[base_index], spline_path.poses[base_index + tolerance]
            (x_ICC, y_ICC, R) = handler.get_circle((p0.pose.position.x, p0.pose.position.y), (p1.pose.position.x, p1.pose.position.y), (p2.pose.position.x, p2.pose.position.y))
            
            # determine initial velocity (v_0) and arc distance (sd) coming from previous waypoint to the current
            v_0 = 0 if index == 0 else linear_speeds[len(linear_speeds) - 1]
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
            # add speeds
            linear_speeds.append(v_1)
            angular_speeds.append(w_1)
    
        return (linear_speeds, angular_speeds)

    def splineDrive(self, spline_path: Path, spline_sd_steps: list, spline_cumulative_sd_steps: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float) -> tuple[Path, list, list, float, float]:
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
        
        :returns a tuple of recorded path of how close the robot got to each waypoint, speeds measured over the recorded path, computed ideal speeds, along with pct error for position (x,y) and heading in radians. 
        (i.e.: tuple[Path=recorded_path, list[tuple[float, float]=(linear_speed, angular_speed)]=recorded_speeds, list[tuple[float, float]=(linear_speed, angular_speed)]=ideal_speeds, float=pct_error_position, float=pct_error_heading])
        """
        
        # current index point considered from a set of points about a base point on a spline path with a radius range of padding
        index, padding = 0, 10
        # feedback control offset from the current index to target when applying PID feedback systems
        feedback_target_offset = 3

        # create angular speed pid feedback handler based on the current index point along a spline
        def feedback_process_variable() -> float:
            orig_x = self.current_pose.pose.position.x
            orig_y = self.current_pose.pose.position.y
            orig_radians = handler.get_heading(self.current_pose)
            
            spline_x = spline_path.poses[min(index + feedback_target_offset, len(spline_path.poses) - 1)].pose.position.x
            spline_y = spline_path.poses[min(index + feedback_target_offset, len(spline_path.poses) - 1)].pose.position.y
            
            variable = handler.rotate(spline_x - orig_x, spline_y - orig_y, -orig_radians)[1]
            return variable

        # compute wheel speeds
        speeds = self.getPathSpeeds(spline_path, spline_sd_steps, spline_cumulative_sd_steps, acceleration, max_linear_speed, max_angular_speed, max_centripetal_acceleration)
        linear_speeds, angular_speeds = speeds[0], speeds[1]
        # pid angular speed feedback controller 
        angular_speed_feedback = PID(kp=self.ANG_KP, ki=self.ANG_KI, kd=self.ANG_KD, process_variable=feedback_process_variable, set_point=lambda: 0.0, clegg_integration=True)
        linear_speed_feedback = PID(kp=self.LIN_KP, ki=self.LIN_KI, kd=self.LIN_KD, process_variable=feedback_process_variable, set_point=lambda: 0.0, clegg_integration=True)

        # path of odometry poses recorded to have minimal error with respect to their corresponding spline path waypoint
        recorded_path = Path()
        # list of 2D tuples of linear and angular speeds at recorded points along driven path
        recorded_speeds = []
        # list of 2D tuples of ideal linear and angular speeds; the recorded speeds are likely to slightly differ
        ideal_speeds = []
        # specified max time allowed to drive from one waypoint to the next before assuming that driving has failed
        frontier_timeout = 1.0 * 1
        # previous stamped time of reaching the next waypoint while driving
        prev_frontier_update_time = -1.0

        # min errors and associated recorded odometry pose compared-to/of a given waypoint 
        # such that the frontier is the current furthest point reached on the spline while driving
        min_position_error, min_heading_error, recorded_pose, frontier_index = None, None, None, -1
        # recorded overall path error while driving
        avg_position_error, avg_heading_error = 0.0, 0.0

        # drive robot with speed data and feedback control
        while index < len(spline_path.poses) - 1:

            # approximate current position (current position; not recorded approximates to waypoints)
            index = self.getPoseIndex(spline_path, self.current_pose, index, padding) 
            # current position error as a vector magnitude at any time
            position_x_pct_error = (self.current_pose.pose.position.x - spline_path.poses[index].pose.position.x) / (2.0 * self.TURTLEBOT3_RADIUS)
            position_y_pct_error = (self.current_pose.pose.position.y - spline_path.poses[index].pose.position.y) / (2.0 * self.TURTLEBOT3_RADIUS)
            position_error = handler.euclid_distance((0,0), (position_x_pct_error, position_y_pct_error))
            
            # check if the robot is taking too long to progress along the path; end path driving if progression/expanding the frontier driven takes too long
            # provided the robot has driven a "sufficient" distance from the starting point; let the sufficient distance be the padding used for approximating current position
            if index > padding and prev_frontier_update_time > 0 and rospy.get_time() - prev_frontier_update_time > frontier_timeout:
                self.setSpeed(0, 0) # stop driving, end early, and return pct performance errors of 100% to indicate faulty path driving
                return (recorded_path, recorded_speeds, ideal_speeds, 1.0, 1.0)

            # record the closest the robot drove to the point and compute error/best init pose of next point
            # if frontier expanded
            if index > frontier_index:
                if recorded_pose is not None: 
                    recorded_path.poses.append(handler.get_pose_stamped(recorded_pose.pose.position.x, recorded_pose.pose.position.y, handler.get_heading(recorded_pose)))
                    recorded_speeds.append((self.current_speed.linear.x, self.current_speed.angular.z))
                    ideal_speeds.append((linear_speeds[index], angular_speeds[index]))
                    # heading error associated with min error (position error) 
                    min_heading_error = abs((handler.get_heading(recorded_pose) - handler.get_heading(spline_path.poses[index])) / self.MAX_SPLINE_TURN)
                    # add an error term to sum for averaging
                    avg_position_error += min_position_error 
                    avg_heading_error += min_heading_error
                    
                prev_frontier_update_time = rospy.get_time() # stamp time of updating frontier
                min_position_error = position_error
                recorded_pose = self.current_pose
                frontier_index = index
                
             # evaluate the closest the robot drove by min error for current point
            elif min_position_error is None or position_error < min_position_error:
                min_position_error = position_error
                recorded_pose = self.current_pose

            # visualize padded local around the robot; visualize by gridcells
            self.rvizViewSplinePathProgression(spline_path, index, padding)
            # look up memoized speed calculations given position index and system feedback control
            ang_speed = angular_speeds[index] + angular_speed_feedback.output() 
            lin_speed = max(0.001, abs(linear_speeds[index]) - max(0, abs(linear_speed_feedback.output())))
            self.setSpeed(lin_speed, ang_speed)

        # come to a stop and return data
        self.setSpeed(0, 0)
        avg_position_error = avg_position_error / len(recorded_path.poses)
        avg_heading_error = avg_heading_error / len(recorded_path.poses)
        return (recorded_path, recorded_speeds, ideal_speeds, avg_position_error, avg_heading_error)

    def driveSplinePath(self, waypoints: list, acceleration: float, max_linear_speed: float, max_angular_speed: float, max_centripetal_acceleration: float) -> tuple[Path, list, list, float, float]:
        """
        Drives a path of splines given waypoints to constrain the spline onto with motion profiling constraints of acceleration [m/sec^2], 
        max linear speed [m/sec], max angular speed [radians/sec], and max centripetal acceleration [m/sec^2]. The recorded path is returned alongside
        percent errors for position and heading.

        :param waypoints [list] The specified list of pose vectors for each waypoint of which the spline path should intersect through
        :param acceleration [float] The specified acceleration in [m/sec^2] to profile by
        :param max_linear_speed [float] The specified max linear speed in [m/sec] to profile by
        :param max_angular_speed [float] The specified max angular speed in [radians/sec] to profile by
        :param max_centripetal_acceleration [float] The specified max centripetal acceleration in [m/sec^2] to profile by

        :returns a tuple of recorded path of how close the robot got to each waypoint, speeds measured over the recorded path, computed ideal speeds, along with pct error for position (x,y) and heading in radians. 
        (i.e.: tuple[Path=recorded_path, list[tuple[float, float]=(linear_speed, angular_speed)]=recorded_speeds, list[tuple[float, float]=(linear_speed, angular_speed)]=ideal_speeds, float=pct_error_position, float=pct_error_heading])
        """
        # add initial robot position to the front of the waypoints and request interpolated spline path. 
        waypoints = [(self.current_pose.pose.position.x, self.current_pose.pose.position.y, handler.get_heading(self.current_pose))] + waypoints
        spline_plan = self.requestSimpleSplinePlan(waypoints)
        # drive spline given computed spline path and motion profiling constraints. 
        recorded_path, recorded_speeds, ideal_speeds, position_error, heading_error = self.splineDrive(spline_plan[0], spline_plan[1], spline_plan[2], acceleration, max_linear_speed, max_angular_speed, max_centripetal_acceleration)
        # return results
        return (recorded_path, recorded_speeds, ideal_speeds, position_error, heading_error)

    def handleSetMotionProfilingService(self, request):
        """
        Represents the service for specifying motion criteria for spline path driving.
        """
        rospy.loginfo("Navigation.py: motion profiling properties service request heard")
        self.ACCELERATION = request.acceleration
        self.MAX_LINEAR_SPEED = request.max_linear_speed
        self.MAX_ANGULAR_SPEED = request.max_angular_speed
        self.MAX_CENTRIPETAL_ACCELERATION = request.max_centripetal_acceleration
        # print specified profiling criteria
        print((self.ACCELERATION, self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION))

    def handleSimulationTestService(self, request):
        """
        Represents the sim testing service that given specified PID coefficients.
        :returns percent errors for both position and heading.  
        """
        rospy.loginfo("Navigation.py: simulation test service request heard")
        # set specified pid coefficients
        self.LIN_KP, self.LIN_KI, self.LIN_KD = request.linear_kp, request.linear_ki, request.linear_kd
        self.ANG_KP, self.ANG_KI, self.ANG_KD = request.angular_kp, request.angular_ki, request.angular_kd
        # run test
        _, position_pct_error, heading_pct_error = self.test()
        return GetNavSimTestResponse(position_error=position_pct_error, heading_error=heading_pct_error)

    def test(self) -> tuple[Path, list, list, float, float]:
        """
        Conducts a spline path driving simulation with the current motion profiling criteria. 
        :returns a tuple of recorded path of how close the robot got to each waypoint, list of recorded speeds, list of ideal speeds, and percent error for position and heading,  
        (i.e.: tuple[Path=recorded_path, list[tuple[float, float]=(linear_speed, angular_speed)]=recorded_speeds, list[tuple[float, float]=(linear_speed, angular_speed)]=ideal_speeds, 
        float=pct_error_position, float=pct_error_heading])
        """
        # waypoints to travel through along spline path: (waypoint = (x, y, radians))
        waypoints = [(4,2,-math.pi/4.0), (5,1,-math.pi/2.0), (4, 0, -math.pi*3.0/4.0), (0, 0, math.pi)]
        recorded_path, recorded_speeds, ideal_speeds, position_pct_error, heading_pct_error = self.driveSplinePath(waypoints, self.ACCELERATION, self.MAX_ANGULAR_SPEED, self.MAX_LINEAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION)
        try:
            self.node_rate.sleep()
        except:
            pass
        # visualize driving performance
        self.rvizViewDrivingProgression(recorded_path)
        return (recorded_path, recorded_speeds, ideal_speeds, position_pct_error, heading_pct_error)

    def internalTest(self) -> tuple[float, float]:
        # absolute maximum centripetal acceleration given simulation physics
        coeff_static_friction = 1.0
        centripetal_acceleration = coeff_static_friction * 9.81 # [m/sec^2]
        # percentage of centripetal acceleration to consider when specifying max centripetal acceleration for spline path driving
        scaler = 0.8
        
        # other motion profiling constraints:
        self.ACCELERATION = 0.2 # [m/sec^2]
        self.MAX_ANGULAR_SPEED = 0.6 # [radians/sec]
        self.MAX_LINEAR_SPEED = 0.75 # [m/sec]
        self.MAX_CENTRIPETAL_ACCELERATION = centripetal_acceleration * scaler # [m/sec^2]
        _, position_pct_error, heading_pct_error = self.test()

        # print results:
        rospy.loginfo("Navigation.py: Internal Testing: Position error: " + format(100.0*position_pct_error, '.5f') + "%, Heading error: " + format(100.0*heading_pct_error, '.5f') + "%")
        return (position_pct_error, heading_pct_error)

    def run(self):
        """
        Runs the Navigation node either with internal testing within the node, or by external testing usually conducted by ROS services with NavAnalyzer. 
        """
        # conduct testing internally if specified so
        if INTERNAL_TESTING: 
            # let internal testing conduct trials of simulations over specified motion profiling criterial and PID feedback coefficients
            epochs = 50
            valid_epochs = epochs
            avg_position_error, avg_heading_error = 0.0, 0.0
            # step through trials/epochs for each sim test
            for epoch in range(0, epochs):
                # conduct trial, reset, and track valid trials to average over at the end of testing
                position_error, heading_error = self.internalTest()
                self.resetRobot()
                # add failed attempts
                avg_position_error += position_error
                avg_heading_error += heading_error
                
                #if position_error < 1 and heading_error < 1:
                #    avg_position_error += position_error
                #    avg_heading_error += heading_error
                #else:
                #    valid_epochs -= 1

            # handle internal testing results based on averaged valid trials
            if valid_epochs > 0:
                avg_position_error /= valid_epochs
                avg_heading_error /= valid_epochs
            else: 
                # no valid trials; assume max error 
                avg_position_error = 1.0
                avg_heading_error = 1.0

            # print results:
            average_error = (avg_position_error + avg_heading_error) / 2.0
            rospy.loginfo("Navigation.py: Internal Testing: Avg Position error: " + format(100.0*position_error, '.5f') + "%, Avg Heading error: " + format(100.0*heading_error, '.5f') + "%, Avg Error: " + format(100.0*average_error, '.5f') + "%")
            rospy.spin()

        motion_criteria_service = rospy.Service("/navigation/set_motion_criteria", GetNavCriteriaPlan, self.handleSetMotionProfilingService)
        nav_sim_test_service = rospy.Service("/navigation/sim_test", GetNavSimTest, self.handleSimulationTestService)
        rospy.spin()

if __name__ == "__main__":
    Navigation().run()
