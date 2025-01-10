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

        # PCT TEST #1
        self.ANG_KP, self.ANG_KI, self.ANG_KD = 15.265625, 0.0000835, 14.829688
        self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.050781, 0.002802, 1.097656  
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
        # initialize pid testing handler functions
        self.init_pid_tuning = True

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
            self.init_pid_tuning = False
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
            self.init_pid_tuning = False
            return error
        
        # pid controller auto-tuners
        angular_pid_tuner = PIDTuner(ang_kp, ang_ki, ang_kd, 0.5, 0.0, 0.60, angular_pid_test)
        linear_pid_tuner = PIDTuner(lin_kp, lin_ki, lin_kd, 3.0, 0.001, 3.0, linear_pid_test)

         # set auto tuner(s) coefficient error logging files 
        root_dir = handler.get_ros_package_root("ROS_SplinePathMotionProfiling")
        rel_pid_err_log_dir = "/pid_tuning_err_logs/"
        angular_pid_tuner.setErrorLog(root_dir + rel_pid_err_log_dir, error_file)
        linear_pid_tuner.setErrorLog(root_dir + rel_pid_err_log_dir, error_file)

        # tune coefficients
        angular_pid_tuner.opt_kp, angular_pid_tuner.opt_ki, angular_pid_tuner.opt_kd = angular_pid_tuner.tune(5, 0, 5, 1, True)
        lin_kp, lin_ki, lin_kd = linear_pid_tuner.tune(10, 5, 10, 1, True)
        ang_kp, ang_ki, ang_kd = angular_pid_tuner.tune(10, 0, 10, 2, True)

        print("Found Coefficients: ")
        print("Angular: kp = " + format(float(ang_kp), '.6f') + ", ki = " + format(float(ang_ki), '.6f') + ", kd = " + format(float(ang_kd), '.6f'))
        print("Linear: kp = " + format(float(lin_kp), '.6f') + ", ki = " + format(float(lin_ki), '.6f') + ", kd = " + format(float(lin_kd), '.6f'))
        return (lin_kp, lin_ki, lin_kd, ang_kp, ang_ki, ang_kd)

    def analyzePerformance(epochs: int):
        # set error logging file
        pass

    def run(self):
        # initialize profiling criteria
        self.configureNavMotionProfilingCriteria(self.ACCELERATION, self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION)
        # tune PID coefficients testing: 
        pid_tuning_error_log = "err_log_pct_C.csv"
        self.tuneFeedbackPID(self.LIN_KP, self.LIN_KI, self.LIN_KD, self.ANG_KP, self.ANG_KI, self.ANG_KD, pid_tuning_error_log)
        rospy.spin()

if __name__ == "__main__":
    NavAnalyzer().run()

