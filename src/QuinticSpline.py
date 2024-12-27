#!/usr/bin/env python3
from __future__ import annotations
import matplotlib.pyplot as plt
import math
import handler

from ROS_SplinePathMotionProfiling.srv import GetSimpleSplinePlan, GetSimpleSplinePlanResponse

from geometry_msgs.msg import PoseStamped
import rospy

# Represents the relative quintic function to compute subpoints from 
class Quintic:
    """
    Represents the relative quintic function to compute given a specified starting and ending waypoint. 
    """    
    def __init__(self, waypoint_0: PoseStamped, waypoint_1: PoseStamped):
        """
        Initializes the Quintic with the necessary coefficients based on the specified waypoints and headings.
        :param waypoint_0 [PoseStamped] The specified initial waypoint 
        :param waypoint_1 [PoseStamped] The specified final waypoint
        """
        # The specified waypoints
        self.waypoint_0 = waypoint_0
        self.waypoint_1 = waypoint_1
        # Function coefficients
        self.A, self.B, self.C, self.D, self.E, self.F = 0, 0, 0, 0, 0, 0
        self.computeQuintic()
            
    # Determines the quintic function coefficients such that the quintic tangentially instersects the specified waypoints
    def computeQuintic(self):
        """
        Computes the coefficients of a quintic polynomial given the initial and final waypoints of the Quintic.
        """
        x_0 = self.waypoint_0.pose.position.x 
        y_0 = self.waypoint_0.pose.position.y
        t_0 = handler.get_heading(self.waypoint_0)
        x_1 = self.waypoint_1.pose.position.x
        y_1 = self.waypoint_1.pose.position.y
        t_1 = handler.get_heading(self.waypoint_1)
        k_0, k_1 = 0, 0
    
        # Coefficient formulas
        self.A = (12*y_0 - 12*y_1 + k_0*pow(x_0,2) + k_0*pow(x_1,2) - k_1*pow(x_0,2) - k_1*pow(x_1,2) - 6*x_0*math.tan(t_0) - 6*x_0*math.tan(t_1) + 6*x_1*math.tan(t_0) + 6*x_1*math.tan(t_1) - 2*k_0*x_0*x_1 + 2*k_1*x_0*x_1)/(2*pow(x_0 - x_1,5));
        self.B = (14*pow(x_0,2)*math.tan(t_0) + 16*pow(x_0,2)*math.tan(t_1) - 16*pow(x_1,2)*math.tan(t_0) - 14*pow(x_1,2)*math.tan(t_1) - 30*x_0*y_0 + 30*x_0*y_1 - 30*x_1*y_0 + 30*x_1*y_1 - 2*k_0*pow(x_0,3) - 3*k_0*pow(x_1,3) + 3*k_1*pow(x_0,3) + 2*k_1*pow(x_1,3) + 4*k_0*x_0*pow(x_1,2) + k_0*pow(x_0,2)*x_1 - k_1*x_0*pow(x_1,2) - 4*k_1*pow(x_0,2)*x_1 + 2*x_0*x_1*math.tan(t_0) - 2*x_0*x_1*math.tan(t_1))/(2*pow(x_0 - x_1,5));
        self.C = -(8*pow(x_0,3)*math.tan(t_0) + 12*pow(x_0,3)*math.tan(t_1) - 12*pow(x_1,3)*math.tan(t_0) - 8*pow(x_1,3)*math.tan(t_1) - k_0*pow(x_0,4) - 3*k_0*pow(x_1,4) + 3*k_1*pow(x_0,4) + k_1*pow(x_1,4) - 20*pow(x_0,2)*y_0 + 20*pow(x_0,2)*y_1 - 20*pow(x_1,2)*y_0 + 20*pow(x_1,2)*y_1 - 4*k_0*pow(x_0,3)*x_1 + 4*k_1*x_0*pow(x_1,3) + 8*k_0*pow(x_0,2)*pow(x_1,2) - 8*k_1*pow(x_0,2)*pow(x_1,2) - 28*x_0*pow(x_1,2)*math.tan(t_0) + 32*pow(x_0,2)*x_1*math.tan(t_0) - 32*x_0*pow(x_1,2)*math.tan(t_1) + 28*pow(x_0,2)*x_1*math.tan(t_1) - 80*x_0*x_1*y_0 + 80*x_0*x_1*y_1)/(2*pow(x_0 - x_1,5));
        self.D = -(k_0*pow(x_1,5) - k_1*pow(x_0,5) + 4*k_0*x_0*pow(x_1,4) + 3*k_0*pow(x_0,4)*x_1 - 3*k_1*x_0*pow(x_1,4) - 4*k_1*pow(x_0,4)*x_1 + 60*x_0*pow(x_1,2)*y_0 + 60*pow(x_0,2)*x_1*y_0 - 60*x_0*pow(x_1,2)*y_1 - 60*pow(x_0,2)*x_1*y_1 - 8*k_0*pow(x_0,2)*pow(x_1,3) + 8*k_1*pow(x_0,3)*pow(x_1,2) + 36*x_0*pow(x_1,3)*math.tan(t_0) - 24*pow(x_0,3)*x_1*math.tan(t_0) + 24*x_0*pow(x_1,3)*math.tan(t_1) - 36*pow(x_0,3)*x_1*math.tan(t_1) - 12*pow(x_0,2)*pow(x_1,2)*math.tan(t_0) + 12*pow(x_0,2)*pow(x_1,2)*math.tan(t_1))/(2*pow(x_0 - x_1,5));
        self.E = (2*pow(x_0,5)*math.tan(t_1) - 2*pow(x_1,5)*math.tan(t_0) + 2*k_0*x_0*pow(x_1,5) - 2*k_1*pow(x_0,5)*x_1 - k_0*pow(x_0,2)*pow(x_1,4) - 4*k_0*pow(x_0,3)*pow(x_1,3) + 3*k_0*pow(x_0,4)*pow(x_1,2) - 3*k_1*pow(x_0,2)*pow(x_1,4) + 4*k_1*pow(x_0,3)*pow(x_1,3) + k_1*pow(x_0,4)*pow(x_1,2) + 60*pow(x_0,2)*pow(x_1,2)*y_0 - 60*pow(x_0,2)*pow(x_1,2)*y_1 + 10*x_0*pow(x_1,4)*math.tan(t_0) - 10*pow(x_0,4)*x_1*math.tan(t_1) + 16*pow(x_0,2)*pow(x_1,3)*math.tan(t_0) - 24*pow(x_0,3)*pow(x_1,2)*math.tan(t_0) + 24*pow(x_0,2)*pow(x_1,3)*math.tan(t_1) - 16*pow(x_0,3)*pow(x_1,2)*math.tan(t_1))/(2*pow(x_0 - x_1,5)); 
        self.F = (2*pow(x_0,5)*y_1 - 2*pow(x_1,5)*y_0 + 10*x_0*pow(x_1,4)*y_0 - 10*pow(x_0,4)*x_1*y_1 - k_0*pow(x_0,2)*pow(x_1,5) + 2*k_0*pow(x_0,3)*pow(x_1,4) - k_0*pow(x_0,4)*pow(x_1,3) + k_1*pow(x_0,3)*pow(x_1,4) - 2*k_1*pow(x_0,4)*pow(x_1,3) + k_1*pow(x_0,5)*pow(x_1,2) - 20*pow(x_0,2)*pow(x_1,3)*y_0 + 20*pow(x_0,3)*pow(x_1,2)*y_1 + 2*x_0*pow(x_1,5)*math.tan(t_0) - 2*pow(x_0,5)*x_1*math.tan(t_1) - 10*pow(x_0,2)*pow(x_1,4)*math.tan(t_0) + 8*pow(x_0,3)*pow(x_1,3)*math.tan(t_0) - 8*pow(x_0,3)*pow(x_1,3)*math.tan(t_1) + 10*pow(x_0,4)*pow(x_1,2)*math.tan(t_1))/(2*pow(x_0 - x_1,5))
    
    # Represents the quintic function dependent on the domain of x
    def f(self, x):
        """
        Represents the quintic function dependent on the domain of x.
        """
        return self.A*pow(x, 5) + self.B*pow(x, 4) + self.C*pow(x, 3) + self.D*pow(x, 2) + self.E*x + self.F
    # Represents the derivative of the quintic function
    def dydx(self, x):
        """
        Represents the derivative of the quintic function on the domain of x.
        """
        return 5*self.A*pow(x, 4) + 4*self.B*pow(x, 3) + 3*self.C*pow(x, 2) + 2*self.D*x + self.E 

class QuinticSplinePath:
    """
    Represents a QuinticSplinePath ROS node that generates a spline path of interpolated waypoints.
    """
    def __init__(self):
        """
        Initializes a QuinticSplinePath node.
        """
        rospy.init_node("quintic_spline_path", anonymous=True)
        self.node_rate = rospy.Rate(10)
        # matplotlib coordinate range: ((min_x, max_x), (min_y, max_y))
        self.plot_range = ((-4, 10), (-3, 10))

        self.path_x, self.path_y = None, None
        self.sd_steps, self.cumulative_sd_steps = None, None

        self.initPublishers()
        self.initSubscribers()
        rospy.sleep(1.0)
    
    def initPublishers(self):
        """
        Initializes and creates all node publishers.
        """
        pass
    def initSubscribers(self):
        """
        Initializes and creates all node subscribers. 
        """
        pass
    
    def handleSplinePlanService(self, request):
        """
        Represents the GetSplinePlan service for a simple spline path that expects a specified Path of waypoints 
        to constrain the spline path to. Returns the interpolated Path of points of the constrained spline. 
        """
        rospy.loginfo("QuinticSpline.py: GetSplinePlan service request heard")
        simple_path = request.waypoints_path
        spline_path_points = self.getInterpolatedSpline(simple_path.poses)
        spline_path = handler.get_path((0, 0), 1.0, spline_path_points[0], spline_path_points[1])
        
        return GetSimpleSplinePlanResponse(spline_path=spline_path, sd_steps=spline_path_points[2], cumulative_sd_steps=spline_path_points[3])

    def plot(self, path_x: list, path_y: list, plot: bool):
        if plot:
            # View interpolated quintic spline plot with matplotlib
            plt.plot(path_x, path_y)
            
            plt.xlabel('X-axis')
            plt.ylabel('Y-axis')
            plt.title('Computed Quintic Spline Path')
            plt.xlim(self.plot_range[0][0], self.plot_range[0][1])
            plt.ylim(self.plot_range[1][0], self.plot_range[1][1])
            plt.axis("square")
            plt.grid(visible=True)
            plt.show()

    def getInterpolatedSpline(self, waypoints: list[PoseStamped]) -> tuple[list, list, list, list]:
        # Create subpoint lists
        path_x, path_y, sd_steps, cumulative_sd_steps = [], [], [], []
        # Iterate between waypoints to generate quintic splines
        for p_index in range(1, len(waypoints)):
            p_0 = waypoints[p_index - 1]
            p_1 = waypoints[p_index]

            if p_index > 1:
                path_x.pop()
                path_y.pop()

            # Find the relative frame of reference by centering the splin on point of origin and a rotation to 
            # parallelize the initial waypoint with the x-axis
            delta_x = p_1.pose.position.x - p_0.pose.position.x
            delta_y = p_1.pose.position.y - p_0.pose.position.y
            # Assuming the initial waypoint is zero, the relative point is the difference after the following rotation with -p_0 heading
            rel_x = delta_x * math.cos(-handler.get_heading(p_0)) - delta_y * math.sin(-handler.get_heading(p_0))
            rel_y = delta_x * math.sin(-handler.get_heading(p_0)) + delta_y * math.cos(-handler.get_heading(p_0))
            rel_theta = handler.get_heading(p_1) - handler.get_heading(p_0)
            # Convert coords
            p_origin, p_rel = PoseStamped(), PoseStamped()
            p_rel.pose.position.x = rel_x
            p_rel.pose.position.y = rel_y
            p_rel.pose.orientation = handler.get_orientation(rel_theta)
            
            # Compute the relative quintic spline function
            q = Quintic(p_origin, p_rel)
            # The specified number of points to partition and interpolate
            partitions = 100
            # The amount to step by through x-domain when interpolating
            partition = rel_x / partitions
            # The initial arc distance for the current spline along the path; the spline created between two waypoints such that
            # the initial arc distance is the cumulative distance of the path traveled up until that point
            init_sd = 0 if len(cumulative_sd_steps) < 1 else cumulative_sd_steps[len(cumulative_sd_steps) - 1]

            # Iterate through all partitions to compute the relative spline; 
            for b_index in range(0, partitions + 1):
                step_x = partition * b_index 
                # Compute derivative to integrate arc distance between points as step size arc distance
                sd = pow(1.0 + pow(q.dydx(step_x), 2.0), 0.5) * partition 
                cumulative_sd = init_sd if b_index == 0 else sd + cumulative_sd_steps[len(cumulative_sd_steps) - 1]
                # Rotate to globabl frame of reference about the origin with +p_0 heading, and translate to restore global spline
                x = step_x * math.cos(handler.get_heading(p_0)) - q.f(step_x) * math.sin(handler.get_heading(p_0)) + p_0.pose.position.x
                y = step_x * math.sin(handler.get_heading(p_0)) + q.f(step_x) * math.cos(handler.get_heading(p_0)) + p_0.pose.position.y
                # Append global path subpoints computed
                path_x.append(x)
                path_y.append(y)  
                sd_steps.append(sd)
                cumulative_sd_steps.append(cumulative_sd)
        self.path_x = path_x
        self.path_y = path_y
        self.sd_steps = sd_steps
        self.cumulative_sd_steps = cumulative_sd_steps
        return (path_x, path_y, sd_steps, cumulative_sd_steps)
    
    def run(self):
        simple_spline_service = rospy.Service("/quintic_spline_path/simple_spline_plan", GetSimpleSplinePlan, self.handleSplinePlanService)
        while not rospy.is_shutdown():
            if self.path_x is not None and self.path_y is not None:
                self.plot(self.path_x, self.path_y, True)
                self.path_x = None
                self.path_y = None
            self.node_rate.sleep()

if __name__ == "__main__":
    QuinticSplinePath().run()
