#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Vector3

class SimulationSpeed:

    def __init__(self):
        # initialize node
        rospy.init_node("simulation_speed", anonymous=True)
        self.node_rate = rospy.Rate(10)
    
    def setSimulationSpeed(self, update_rate: float):
        """"
        Requests and updates physics properities related to Gazebo simulation speed.
        :param update_rate [float] The specified rate 
        """
        rospy.loginfo("SimulationSpeed.py: Requesting physics properties from Gazebo")
        rospy.wait_for_service("/gazebo/set_physics_properties")

        try:
            set_physics_client = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
            get_physics_client = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
            # get physics properties
            properties = get_physics_client()

            # update real-tome update rate for current properties
            properties.time_step = 0.001
            properties.max_update_rate = update_rate

            # set new physics properties
            set_physics_client(
                time_step=properties.time_step,
                max_update_rate=properties.max_update_rate, 
                gravity=properties.gravity,
                ode_config=properties.ode_config
            )

            rospy.loginfo("SimulationSpeed.py: Successfuly updated simulation update rate to " + str(update_rate))
        except rospy.ServiceException as e:
            rospy.logerr("SimulationSpeed.py exception thrown: service call failed => exception: " + e.__str__())

    def run(self):
        update_rate = 1000
        self.setSimulationSpeed(update_rate)
        rospy.spin()

if __name__ == "__main__":
    SimulationSpeed().run()