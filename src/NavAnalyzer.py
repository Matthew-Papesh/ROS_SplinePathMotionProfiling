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

    # LAST TUNING CONVERGING PIDS:
    # LIN => PID=( 0.32227, 0.0010875, 1.3242 )
    # ANG => PID=( 4.2585, 0.00074279, 0.050625 )

    # Sim Test Results (MSE) for accel=0.02, max-lin-speed=1.0, max-ang-speed=1.0, max-centrip-accel=7.84, LIN-PID = [kp=1.0,ki=0,kd=0.25], ANG-PID = [kp=4.0,ki=0.001,kd=15.0]
    # Position error: 0.0002166328170781128, heading error: 1.9915409523919663
    # Position error: 0.00010015798099621572, heading error: 0.29327870231559616
    # Position error: 0.00011856977120692698, heading error: 0.28940784336306236

    # Position error: 0.0001606594634831609, heading error: 0.29074097147422684
    # Position error: 0.0001448117502408014, heading error: 0.4874795499346864

    # Sim Test Results (NMSE):
    # Position error: 0.059612820585739454, heading error: 0.01233413721252712


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
        self.ACCELERATION = 0.02 # [m/sec^2]
        self.MAX_ANGULAR_SPEED = 1.0 # [radians/sec]
        self.MAX_LINEAR_SPEED = 1.0 # [m/sec]
        self.MAX_CENTRIPETAL_ACCELERATION = centripetal_acceleration * scaler # [m/sec^2]
        
        # tuned ANG pid (input = (kp=4.0,ki=0.0001,kd=4.0)) output = (kp=5.196,ki=0.000734,kd=1.035) [NO LIN pid]
        # pid feedback coefficients (linear and angular differential speed PID)
        self.ANG_KP, self.ANG_KI, self.ANG_KD = 5.196, 0.000734, 1.035
        self.LIN_KP, self.LIN_KI, self.LIN_KD = 1.0, 0.001, 0.5 
        
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
        Resets robot position and speeds.
        """
        rospy.loginfo("NavAnalyzer.py: Requesting robot reset in Gazebo from \'/gazebo/set_model_state'\' service")
        rospy.wait_for_service("/gazebo/set_model_state")
        try: 
            client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            # Define the new state
            state_msg = ModelState()
            state_msg.model_name = 'robot' 
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = 0.0
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.x = 0.0
            state_msg.pose.orientation.y = 0.0
            state_msg.pose.orientation.z = 0.0
            state_msg.pose.orientation.w = 1.0
            state_msg.twist.linear.x = 0.0
            state_msg.twist.linear.y = 0.0
            state_msg.twist.linear.z = 0.0
            state_msg.twist.angular.x = 0.0
            state_msg.twist.angular.y = 0.0
            state_msg.twist.angular.z = 0.0
            state_msg.reference_frame = 'world'

            response = client(state_msg)
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to reset robot state with service")
                exit()
            rospy.loginfo("NavAnalyzer.py: robot reset service ended successfully")
        except rospy.ServiceException as e:
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())

    def resetGazebo(self):
        """
        Requests a Gazebo World Simulation.
        """
        rospy.loginfo("NavAnalyzer.py: Requesting Gazebo Reset from \'/gazebo/reset_simulation\' service")
        rospy.wait_for_service("/gazebo/reset_simulation")
        try:
            client = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
            response = client()
            if response is None:
                rospy.logerr("NavAnalyzer.py: error: failed to retrieve world reset service response")
                exit()
            rospy.loginfo("NavAnalyzer.py: world reset service ended successfuly")
        except rospy.ServiceException as e:
            rospy.logerr("NavAnalyzer.py: exception thrown: service call failed => exception: " + e.__str__())

    def run(self):
        # initialize profiling criteria
        self.configureNavMotionProfilingCriteria(self.ACCELERATION, self.MAX_LINEAR_SPEED, self.MAX_ANGULAR_SPEED, self.MAX_CENTRIPETAL_ACCELERATION)

        # test to complete to compute angular pid error
        def angular_pid_test(kp: float, ki: float, kd: float) -> float:
            self.resetGazebo()
            try:
                rospy.sleep(1)
            except: 
                rospy.logwarn("NavAnalyzier.py: Warning: rospy.sleep() failed likely due to ros time rollback from sim reset")
            pos_err, ang_err = self.requestNavSimTest(lin_kp=self.LIN_KP, lin_ki=self.LIN_KI, lin_kd=self.LIN_KD, ang_kp=kp, ang_ki=ki, ang_kd=kd)
            error = (pos_err + ang_err) / 2.0
            print("ANG: Position error: " + format(100.0*pos_err, '.3') + "%, Heading error: " + format(100.0*ang_err, '.3') + "%")
            return error
        # test to complete to compute linear pid error
        def linear_pid_test(kp: float, ki: float, kd: float) -> float:
            self.resetGazebo()
            try:
                rospy.sleep(1)
            except: 
                rospy.logwarn("NavAnalyzier.py: Warning: rospy.sleep() failed likely due to ros time rollback from sim reset")
            pos_err, ang_err = self.requestNavSimTest(lin_kp=kp, lin_ki=ki, lin_kd=kd, ang_kp=self.ANG_KP, ang_ki=self.ANG_KI, ang_kd=self.ANG_KD)
            error = (pos_err + ang_err) / 2.0
            print("LIN: Position error: " + format(100.0*pos_err, '.3') + "%, Heading error: " + format(100.0*ang_err, '.3') + "%")
            return error
        
        # pid controller auto-tuners
        angular_pid_tuner = PIDTuner(self.ANG_KP, self.ANG_KI, self.ANG_KD, 1.0, 0.0001, 2.0, angular_pid_test)
        linear_pid_tuner = PIDTuner(self.LIN_KP, self.LIN_KI, self.LIN_KD, 1.0, 0.0001, 1.0, linear_pid_test)

        # set auto tuner(s) coefficient error logging files 
        root_dir = handler.get_ros_package_root("ROS_SplinePathMotionProfiling")
        rel_pid_err_log_dir = "/pid_tuning_err_logs/"
        angular_pid_tuner.setErrorLog(root_dir + rel_pid_err_log_dir, "ang_kp_err_log.csv", "ang_ki_err_log.csv", "ang_kd_err_log.csv")
        linear_pid_tuner.setErrorLog(root_dir + rel_pid_err_log_dir, "lin_kp_err_log.csv", "lin_ki_err_log.csv", "lin_kd_err_log.csv")

        # tune coefficients
        self.ANG_KP, self.ANG_KI, self.ANG_KD = angular_pid_tuner.tune(10, 10, 10, True)
        self.LIN_KP, self.LIN_KI, self.LIN_KD = linear_pid_tuner.tune(10, 10, 10, True)
        self.ANG_KP, self.ANG_KI, self.ANG_KD = angular_pid_tuner.tune(4, 4, 4, True)

        print("Found Coefficients: ")
        print("Angular: kp = " + format(self.ANG_KP, '.4') + ", ki = " + format(self.ANG_KI, '.4') + ", kd = " + format(self.ANG_KD, '.4'))
        print("Linear: kp = " + format(self.LIN_KP, '.4') + ", ki = " + format(self.LIN_KI, '.4') + ", kd = " + format(self.LIN_KD, '.4'))
        rospy.spin()

if __name__ == "__main__":
    NavAnalyzer().run()

