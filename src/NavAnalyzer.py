#!/usr/bin/env python3 
import rospy
from std_srvs.srv import Empty
from ROS_SplinePathMotionProfiling.srv import GetNavCriteriaPlan, GetNavSimTest
from PIDTuner import PIDTuner
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import handler
import time

class NavAnalyzer: 

    def __init__(self):
        # initialize node
        rospy.init_node("nav_analyzer", anonymous=True)
        self.node_rate = rospy.Rate(10)
        
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
        # initializer flags for auto-tuning pid tests; used upon initializing by evaluating PID coefficients in PIDTuner the first time by a callable error supplier on performance error
        self.init_pid_tuning = True

        # PCT TEST #1 - AP15_152344_AI0_000083_AD15_357032
        # initial values before refining:
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.265625, 0.0000835, 14.829688
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656  
        # Found Coefficients: 
        # Angular: kp = 15.152344, ki = 0.000083, kd = 15.357032
        # Linear: kp = 1.050781, ki = 0.002802, kd = 1.097656
        # NavAnalyzer.py: Performance Analysis => epoch=50 of 50 : Position error: 9.85849%, Heading error: 18.78861%, Moving Avgs: ( 29.73125% , 35.29530% )
        # NavAnalyzer.py: Performance Analysis => epoch=100 of 100 : Position error: 100.00000%, Heading error: 100.00000%, Moving Avgs: ( 32.44348% , 38.88380% )
        
        # PCT TEST #2
        self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.152344, 0.000083, 15.357032
        self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656
        # Found Coefficients: 
        # Angular: kp = 14.990235, ki = 0.000083, kd = 15.184766
        # Linear: kp = 1.050781, ki = 0.002802, kd = 1.097656

        # PCT TEST #3 - AP15_097657_AI0_000083_AD14_949805
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 14.990235, 0.000083, 15.184766
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656
        # Found Coefficients: 
        # Angular: kp = 15.097657, ki = 0.000083, kd = 14.949805
        # Linear: kp = 1.050781, ki = 0.002802, kd = 1.097656
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.097657, 0.000083, 14.949805
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656
        # NavAnalyzer.py: Performance Analysis => epoch=50 of 50 : Position error: 6.45849%, Heading error: 16.09992%, Moving Avgs: ( 14.53449% , 21.72352% )
        # NavAnalyzer.py: Performance Analysis => epoch=100 of 100 : Position error: 5.46207%, Heading error: 22.25156%, Moving Avgs: ( 33.21671% , 38.87678% )

        # PCT TEST LINEAR #1 - LP0_847656_LI0_002552_LD0_529297
        # Found Coefficients: 
        # Angular: kp = 15.097657, ki = 0.000083, kd = 14.949805
        # Linear: kp = 0.847656, ki = 0.002552, kd = 0.529297
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.097657, 0.000083, 14.949805
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 0.847656, 0.002552, 0.529297
        # NavAnalyzer.py: Performance Analysis => epoch=50 of 50 : Position error: 9.10219%, Heading error: 15.25195%, Moving Avgs: ( 14.76538% , 22.19665% )
        # NavAnalyzer.py: Performance Analysis => epoch=50 of 50 : Position error: 7.76359%, Heading error: 22.61554%, Moving Avgs: ( 7.49987% , 17.89974% )

        # no PID:
        #self.ANG_KP, self.ANG_KI, self.ANG_KD = 0.0, 0.0, 0.0
        #self.LIN_KP, self.LIN_KI, self.LIN_KD = 0.0, 0.0, 0.0

    def configureNavMotionProfilingCriteria(self, acceleration: float, max_lin_speed: float, max_ang_speed: float, max_centripetal_acceleration: float): 
        """
        Configures motion profiling criteria for spline path driving navigation given a fixed acceleration rate and upper bounded motion constraints. 
        param: acceleration [float] The specified fixed acceleration in [m/sec^2]
        param: max_lin_speed [float] The specified max linear speed in [m/sec]
        param: max_ang_speed [float] The specified max angular speed in [radians/sec]
        param: max_centripetal_acceleration [float] The specified max centripetal acceleration in [m/sec^2] 
        """
        rospy.loginfo("NavAnalyzer.py: Requesting motion profiling criteria config from \'/navigation/set_motion_criteria\' service")
        rospy.wait_for_service("/navigation/set_motion_criteria")
        try: 
            client = rospy.ServiceProxy("/navigation/set_motion_criteria", GetNavCriteriaPlan)
            response = client(acceleration=acceleration, max_linear_speed=max_lin_speed, max_angular_speed=max_ang_speed, max_centripetal_acceleration=max_centripetal_acceleration)
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to configure motion profiling criteria; service may have failed")
                exit()
            rospy.loginfo("NavAnalyzer.py: motion profiling criteria config service ended successfully")
        except rospy.ServiceException as e:
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())

    def requestNavSimTest(self, lin_kp: float, lin_ki: float, lin_kd: float, ang_kp: float, ang_ki: float, ang_kd: float):
        """
        Requests a navigation simulation test for spline driving to be run given specified navigation feedback PID coefficients to return the position and heading error
        by comparing the recorded positioning of the robot against the spline path being followed. 
        param: lin_kp [float] The specified proportional coefficient for linear speed feedback control
        param: lin_ki [float] The specified integral coefficient for linear speed feedback control
        param: lin_kd [float] The specified derivative coefficient for linear speed feedback control
        param: ang_kp [float] The specified proportional coefficient for angular speed feedback control
        param: ang_ki [float] The specified integral coefficient for angular speed feedback control
        param: ang_kd [float] The specified derivative coefficient for angular speed feedback control
        returns: The position error and heading error of the overall path driven compared to spline path followed
        """
        rospy.loginfo("NavAnalyzer.py: Requesting navigation simulation test from \'/navigation/sim_test\' service")
        rospy.wait_for_service("/navigation/sim_test")
        try:
            client = rospy.ServiceProxy("/navigation/sim_test", GetNavSimTest)
            response = client(linear_kp=lin_kp, linear_ki=lin_ki, linear_kd=lin_kd, angular_kp=ang_kp, angular_ki=ang_ki, angular_kd=ang_kd)
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to retrieve successful test results")
                exit()
            rospy.loginfo("NavAnalyzer.py: nav sim test service ended; returning test results")
            return (response.position_error, response.heading_error)
        except rospy.ServiceException as e: 
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())
        return None
            
    def resetRobot(self):
        """
        Requests a reset of the robot in Gazebo World Simulation.
        """
        rospy.loginfo("NavAnalyzer.py: Requesting Robot Reset from \'/gazebo/set_model_state\' service")
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
                rospy.logerr("NavAnalyzer.py: error: failed to reset robot on service response")
                exit()
            rospy.loginfo("NavAnalyzer.py: robot reset service ended successfuly")
        except rospy.ServiceException as e:
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())

    def logPerformanceError(self, root_dir: str, error_file: str, position_error_data: list, heading_error_data: list): 
        """
        Writes position and heading error from navigation simulation tests such that the i-th position and heading error describes 
        the average percent error across all waypoints for that spline path driven. 
        param: root_dir [str] The specified root directory
        param: error_file [str] The specified error file to write to
        param: position_error_data [list] The specified position error distribution
        param: heading_error_data [list] The specified heading error distribution 
        """
        error_file = root_dir + error_file
        count = min(len(position_error_data), len(heading_error_data))
        with open(error_file, "a") as file:
            for i in range(0, count):
                file.write(str(position_error_data[i]) + ", " + str(heading_error_data[i]) + "\n")

    def tuneFeedbackPID(self, lin_kp: float, lin_ki: float, lin_kd: float, ang_kp: float, ang_ki: float, ang_kd: float, error_file: str):
        """
        Tunes and refines feedback PID coefficients for linear and angular speed feedback control given specified initial coefficients to start with when tuning; 
        prints out tuning results and returns them as a vector in the same order as the parameters are given. 
        param: ang_kp [float] The specified angular speed proportional coefficient
        param: ang_ki [float] The specified angular speed integral coefficient
        param: ang_kd [float] The specified angular speed derivative coefficient
        param: lin_kp [float] The specified linear speed proportional coefficient
        param: lin_ki [float] The specified linear speed integral coefficient
        param: lin_kd [float] The specified linear speed derivative coefficient
        param: error_file [str] The specified error file to log performance error to
        returns: the discovered tuned coefficients
        """
        # initialize profiling criteria
        self.configureNavMotionProfilingCriteria(self.ACCELERATION, self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION)
    
        # test to complete to compute angular pid error
        def angular_pid_test(kp: float, ki: float, kd: float) -> float:
            # intialize, wait, and reset
            self.resetRobot()
            try:
                rospy.sleep(1)
            except: 
                rospy.logwarn("NavAnalyzer.py: Warning: rospy.sleep() failed likely due to ros time rollback from sim reset")
            # simulate test for averaged error
            pos_err, ang_err = self.requestNavSimTest(lin_kp=lin_kp, lin_ki=lin_ki, lin_kd=lin_kd, ang_kp=kp, ang_ki=ki, ang_kd=kd)
            error = (pos_err + ang_err) / 2.0

            # output status and return error results
            status = "NavAnalyzer.py: ANG: Position error: " + format(100.0*pos_err, '.5f') + "%, Heading error: " + format(100.0*ang_err, '.5f') + "%, Avg error: " + format(100.0*error, '.5f') + "%"
            rospy.loginfo(f"{handler.str_bold_start}{status}{handler.str_bold_end}")
            return error
        # test to complete to compute linear pid error
        def linear_pid_test(kp: float, ki: float, kd: float) -> float:
            # initialize, wait, and reset
            self.resetRobot()
            try:
                rospy.sleep(1)
            except: 
                rospy.logwarn("NavAnalyzer.py: Warning: rospy.sleep() failed likely due to ros time rollback from sim reset")
            # simulate test for averaged error
            pos_err, ang_err = self.requestNavSimTest(lin_kp=kp, lin_ki=ki, lin_kd=kd, ang_kp=ang_kp, ang_ki=ang_ki, ang_kd=ang_kd)
            error = (pos_err + ang_err) / 2.0

            # output status and return error results 
            status = "NavAnalyzer.py: LIN: Position error: " + format(100.0*pos_err, '.5f') + "%, Heading error: " + format(100.0*ang_err, '.5f') + "%, Avg error: " + format(100.0*error, '.5f') + "%"
            rospy.loginfo(f"{handler.str_bold_start}{status}{handler.str_bold_end}")
            return error
        
        # pid controller auto-tuners
        angular_pid_tuner = PIDTuner(ang_kp, ang_ki, ang_kd, 0.5, 0.0, 0.60, angular_pid_test)
        linear_pid_tuner = PIDTuner(lin_kp, lin_ki, lin_kd, 0.5, 0.001, 0.75, linear_pid_test)

         # set auto tuner(s) coefficient error logging files 
        root_dir = handler.get_ros_package_root("ROS_SplinePathMotionProfiling")
        rel_pid_err_log_dir = "/pid_tuning_err_logs/"
        angular_pid_tuner.setErrorLog(root_dir + rel_pid_err_log_dir, error_file)
        linear_pid_tuner.setErrorLog(root_dir + rel_pid_err_log_dir, error_file)

        # tune coefficients
        #ang_kp, ang_ki, ang_kd = angular_pid_tuner.tune(5, 0, 5, 1, True)
        lin_kp, lin_ki, lin_kd = linear_pid_tuner.tune(8, 3, 8, 1, True)
        #ang_kp, ang_ki, ang_kd = angular_pid_tuner.tune(10, 0, 10, 2, True)

        print("Found Coefficients: ")
        print("Angular: kp = " + format(float(ang_kp), '.6f') + ", ki = " + format(float(ang_ki), '.6f') + ", kd = " + format(float(ang_kd), '.6f'))
        print("Linear: kp = " + format(float(lin_kp), '.6f') + ", ki = " + format(float(lin_ki), '.6f') + ", kd = " + format(float(lin_kd), '.6f'))
        return (lin_kp, lin_ki, lin_kd, ang_kp, ang_ki, ang_kd)

    def analyzePerformance(self, epochs: int, error_file: str):
        """
        Analyzes position and heading error and collects both per epoch as a record in the error file specified as pair of error distributions.
        param: epochs [int] The specified number of data points to collect from this number of test simulations to run
        """
        # performance error loggers
        position_error_log = []
        heading_error_log = []
        # performance error sums
        position_error_sum = 0.0
        heading_error_sum = 0.0

        for epoch in range(0, epochs):
            # initialize, reset, and wait
            self.resetRobot()
            try:
                rospy.sleep(1)
            except: 
                rospy.logwarn("NavAnalyzer.py: Warning: rospy.sleep() failed likely due to ros time rollback from sim reset")
            # simulate and test for averaged errors
            pos_err, ang_err = self.requestNavSimTest(lin_kp=self.LIN_KP, lin_ki=self.LIN_KI, lin_kd=self.LIN_KD, ang_kp=self.ANG_KP, ang_ki=self.ANG_KI, ang_kd=self.ANG_KD)
            
            # log errors
            position_error_log.append(pos_err)
            heading_error_log.append(ang_err)
            position_error_sum += pos_err
            heading_error_sum += ang_err
            # output status with moving averages
            pos_err_pct, ang_err_pct = format(100.0*pos_err, '.5f'), format(100.0*ang_err, '.5f')
            avg_pcts = "( " + format(100.0*(position_error_sum / float(epoch + 1)), '.5f') + "% , " + format(100.0*(heading_error_sum / float(epoch + 1)), '.5f') + "% )"
            status = "NavAnalyzer.py: Performance Analysis => epoch=" + str(epoch + 1) + " of " + str(epochs) + " : Position error: " + pos_err_pct + "%, Heading error: " + ang_err_pct + "%, Moving Avgs: " + avg_pcts
            rospy.loginfo(f"{handler.str_bold_start}{status}{handler.str_bold_end}")

        # set error logging file
        root_dir = handler.get_ros_package_root("ROS_SplinePathMotionProfiling")
        rel_err_log_dir = "/spline_err_logs/"
        self.logPerformanceError(root_dir + rel_err_log_dir, error_file, position_error_log, heading_error_log)

    def run(self):
        # initialize profiling criteria
        self.configureNavMotionProfilingCriteria(self.ACCELERATION, self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION)
        
        # tune PID coefficients testing: 
        #pid_tuning_error_log = "err_log_pct_D.csv"
        #self.tuneFeedbackPID(self.LIN_KP, self.LIN_KI, self.LIN_KD, self.ANG_KP, self.ANG_KI, self.ANG_KD, pid_tuning_error_log)
        
        # analyze performance:    err_log_feedback_AP15_097657_AI0_000083_AD14_949805
        #performance_error_log = "err_log_feedback_AP15_097657_AI0_000083_AD14_949805.csv" # with tuning #3 results
        performance_error_log = "err_log_feedback_AP15_152344_AI0_000083_AD15_357032.csv" # with tuning #1 results
        #performance_error_log = "err_log_feedback_LP0_847656_LI0_002552_LD0_529297.csv"
        #performance_error_log = "err_log_feedback_INIT_PID.csv"
        #performance_error_log = "err_log_feedback_NO_PID.csv"
        self.analyzePerformance(50, performance_error_log) 
        rospy.spin()

if __name__ == "__main__":
    NavAnalyzer().run()

