import signal
import math
import numpy as np

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan



desired_angle = 90
initial_dist_list = []


class Tb3(Node):
    def __init__(self):
        super().__init__("tb3")

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 1  # message type  # topic name
        )  # history depth

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data,
        )  # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.flag = 0
        

    def vel(self, lin_vel_percent, ang_vel_percent):
        """Publishes linear and angular velocities in percent"""
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def rotate(self,point: tuple[float, float], angle: float):
        from math import cos, sin

        return (
            point[0] * cos(angle) - point[1] * sin(angle),
            point[1] * cos(angle) + point[0] * sin(angle),
        )


    def translate(self,point: tuple[float, float], translation: tuple[float, float]):
        return point[0] + translation[0], point[1] + translation[1]


    def lds_to_robot(self,point: tuple[float, float]) -> tuple[float, float]:
        ROBOT_TO_LDS_TRANSLATION = [-0.07, 0]
        return self.translate(point, ROBOT_TO_LDS_TRANSLATION)


    def lds_distance_to_point_in_lds(self,distance: float, angle: float) -> tuple[float, float]:
        """Angle is in radian"""
        from math import cos, sin

        return distance * cos(angle), distance * sin(angle)
    
    def rotate_list(self,l,rotation_factor):
        return l[rotation_factor:] + l[:rotation_factor]
    
    def rotation_error(self,ranges_actual: list, ranges_desired: list) -> float:
        
        ranges_actual = np.array(ranges_actual)
        ranges_desired = np.array(ranges_desired)
        

        result = [True for _ in range(n)]
        mean_values = [2 for _ in range(n)]            

        for i in range(n):            
            positions = i % n  # Ensure positions is within range [0, n)
            ranges_actual_rotated = np.concatenate((ranges_actual[-positions:], ranges_actual[:-positions]))            
            mean_values[i] = np.nanmean(np.abs(ranges_actual_rotated - np.resize(ranges_desired,n)))
            
        
        min_mean = np.min(mean_values)
        #print(min(mean_values))              
        index = np.where(mean_values == min_mean)[0][0]
        
        if (index > n/2):
            error = n - index 
        else:
            error = -index

        print(error)
        
        # raise NotImplementedError
        return error

    
    def control(self,error: float) -> float:
        """Returns the value used to drive the motors in percent based on the
        error. Error is the difference between actual and desired value. Error may
        be also negative. """

        p = 400
        if (error > 50):
                p = 10 + np.abs(400 - (error - 50) * 10 )        
        velocity_percentage = error / n * p
        return velocity_percentage


    




    def scan_callback(self, msg):
        """Is run whenever a LaserScan msg is received"""
        #print()
        #print("Distances:")
        global n
        n = len(msg.ranges)

        
        current_dist_list = []
        desired_dist_list = []

        robot_initial = []
        robot_current = []
        robot_desired = []

        if self.flag==0:
            global initial_dist_list
            initial_dist_list = msg.ranges
            self.flag=self.flag + 1

        
        current_dist_list = msg.ranges
        
        # robot_init_pts = ()
        # robot_curr_pts = ()

        for i in range(n):
            robot_init_pts = self.lds_to_robot(self.lds_distance_to_point_in_lds(initial_dist_list[i],np.radians(i*360/n)))
            robot_initial.append(math.sqrt(robot_init_pts[0]**2 + robot_init_pts[1]**2))            

            robot_curr_pts = self.lds_to_robot(self.lds_distance_to_point_in_lds(current_dist_list[i],np.radians(i*360/n)))
            robot_current.append(math.sqrt(robot_curr_pts[0]**2 + robot_curr_pts[1]**2))
        
        #print(len(robot_initial))
        robot_desired = self.rotate_list(robot_initial,desired_angle)
        

        error = self.rotation_error(robot_current,robot_desired)

        ctr = self.control(error)

        self.vel(0, ctr)



        # print("⬆️ :", msg.ranges[0])
        # print("⬇️ :", msg.ranges[n // 2])
        # print("⬅️ :", msg.ranges[n // 4])
        # print("➡️ :", msg.ranges[-n // 4])


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print("Waiting for messages...")
    print("If you cannot kill the process using CTRL+C, use CTRL+\\")

    def stop_robot(sig, frame):
        tb3.vel(0, 0)
        tb3.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop_robot)  # Stop on SIGINT
    rclpy.spin(tb3)


if __name__ == "__main__":
    main()