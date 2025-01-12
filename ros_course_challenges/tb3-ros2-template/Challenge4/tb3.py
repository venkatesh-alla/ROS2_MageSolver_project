import signal
import math
import numpy as np

import rclpy  # ROS client library
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from enum import Enum,auto



class State(Enum):
    TOP_OR_LEFT_WALL = auto()
    BOTTOM_OR_RIGHT_WALL = auto()
    STOP = auto()

    #ODOM_WAIT_STATE = auto()
    ODOM_WORKING = auto()
    #LASER_WAIT_STATE = auto()
    LASER_WORKING = auto()



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
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',  # TurtleBot3 publishes odometry messages on this topic
            self.odom_callback,
            10
        )
        self.odom_sub

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.initial_flag = 0
        self.initial_pose = []
        self.index = 1
     
        self.st_wait = State.LASER_WORKING
        self.st = State.TOP_OR_LEFT_WALL


    
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

    def odom_callback(self,msg):

        if self.st_wait == State.ODOM_WORKING:

            def rotate(point: tuple[float, float], angle: float):
                from math import cos, sin

                return (
                    point[0] * cos(angle) - point[1] * sin(angle),
                    point[1] * cos(angle) + point[0] * sin(angle),
                )


            def translate(point: tuple[float, float], translation: tuple[float, float]):
                return point[0] + translation[0], point[1] + translation[1]


            def odom_to_zero(current_pose_in_odom):            
                # Reverse of `zero_to_odom`
                current_pose_in_zero = []
                from math import fmod, pi
                def minus(point):
                    return -point[0], -point[1]
                
                point = (current_pose_in_odom[0],current_pose_in_odom[1])
                start_pos_in_odom = (self.initial_pose[0],self.initial_pose[1])
                start_ori_in_odom = self.initial_pose[2]
                point_in_zero = rotate(translate(point, minus(start_pos_in_odom)),-start_ori_in_odom,)

                #orientation_in_zero = fmod(current_pose_in_odom[2] - start_ori_in_odom,pi)
                # if start_ori_in_odom<0:
                #     orientation_in_zero = current_pose_in_odom[2] + start_ori_in_odom
                # else:
                #     orientation_in_zero = current_pose_in_odom[2] - start_ori_in_odom
                orientation_in_zero = current_pose_in_odom[2] - start_ori_in_odom
                
                current_pose_in_zero = list(point_in_zero)
                current_pose_in_zero.append(orientation_in_zero)
                #print(current_pose_in_zero)       
                        
                return current_pose_in_zero
            
            def zero_to_robot_odom(current_pose_in_zero):
                # Converting the values with respect to the odom_zero to the values as seen by the robot
                current_pose_in_robot = []
                from math import fmod, pi
                def minus(point):
                    return -point[0], -point[1]
                
                point = (current_pose_in_zero[0],current_pose_in_zero[1])
                start_pos_in_zero = (self.initial_pose[0],self.initial_pose[1])
                start_ori_in_zero = self.initial_pose[2]
                point_in_robot = rotate(translate(point, minus(start_pos_in_zero)),-start_ori_in_zero,)

                orientation_in_robot = fmod(current_pose_in_zero[2] - start_ori_in_zero, pi)
                
                current_pose_in_robot = list(point_in_robot)
                current_pose_in_robot.append(orientation_in_robot)
                print(current_pose_in_robot)
                return

            
            def control(current_pose_in_zero,goal_pose_in_zero,goal_pose2_in_zero):
                
                # Gives the value of the control percentage     
            
                return
            
            def robot_translation(current_pose_in_zero,goal_pose_in_zero):

                # minimises the distance between current pose and goal pose

                distance = 0 # Initialising distance value
                [x1,y1,theta1] = current_pose_in_zero
                [x2,y2,theta2] = goal_pose_in_zero

                distance = math.sqrt((x2 - x1)**2 + (y2-y1)**2)
                #print(f"Distance:{distance}")
                if distance > 0.10:
                    self.vel(40,0)
                    
                else:
                    self.vel(0,0)
                    distance = 0
                    #self.st = State.ROTATING
                    self.index+=1
                
                return

                
            
            def robot_rotation_clockwise(current_pose_in_zero,goal_pose_in_zero):

                # minimises the angular error
                
                [x1,y1,theta1] = current_pose_in_zero
                [x2,y2,theta2] = goal_pose_in_zero

                angular_error = abs(theta1 - theta2)
                if angular_error > 0.05:
                    print(angular_error)
                    self.vel(0,-10)
                else:
                    self.vel(0,0)
                    #self.st = State.TO_THE_SECOND_WALL 
                    self.index+=1
                return 
            
            def robot_rotation_anticlockwise(current_pose_in_zero,goal_pose_in_zero):

                # minimises the angular error
                
                [x1,y1,theta1] = current_pose_in_zero
                [x2,y2,theta2] = goal_pose_in_zero

                angular_error = abs(theta1 - theta2)
                if angular_error > 0.05:
                    print(angular_error)
                    self.vel(0,10)
                else:
                    self.vel(0,0)
                    #self.st = State.TO_THE_SECOND_WALL 
                    self.index+=1
                return
            
           

            
            
            #print("Current state is Odom working")
            #print(self.st)

            # Access position and orientation from the odometry message
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation

            position = [pos.x,pos.y,pos.z]
            orientation = [orient.w,orient.x,orient.y,orient.z]

            # conversion of orientation from quaternions to euler
            orient_euler = []
            orient_euler = quat2euler(orientation)


            pose = [pos.x,pos.y,orient_euler[2]]       

            
            #print(f"Position: [x={position[0]}, y={position[1]},z={position[2]}]") # Position in x,y and z
            #print(f"Orientation: [x={orient_euler[0]}, y={orient_euler[1]},z={orient_euler[2]}]") # orientation in Euler
            
            #print(f"pose:[x={pose[0]},y={pose[1]}], angle:{pose[2]}")

            # Storing the initial pose in a list
            if self.initial_flag == 0:
                self.initial_pose = pose
                #print(f"initial pose: x:{initial_pose[0]}, y:{initial_pose[1]}, angle:{initial_pose[2]}")
                self.initial_flag += 1
                #odom_to_zero([1,1,initial_pose[2]])

            current_pose_in_zero = odom_to_zero(pose)
                      
            # if self.st == State.TO_THE_FIRST_WALL:
            #     robot_translation(current_pose_in_zero,goal_pose_in_zero)
                
            
            # if self.st == State.ROTATING:
            #     robot_rotation(current_pose_in_zero,goal_pose_in_zero)
                

            # if self.st == State.TO_THE_SECOND_WALL:
            #     robot_translation(current_pose_in_zero,goal_pose2_in_zero)
            if self.st == State.BOTTOM_OR_RIGHT_WALL:
                # TRT
                goal1_in_zero = [1,0,0]
                goal2_in_zero = [1,0,-math.pi/2]
                goal3_in_zero = [1,-1,-math.pi/2]

                if self.index == 1:
                    robot_translation(current_pose_in_zero,goal1_in_zero)
                
                if self.index == 2:
                    robot_rotation_clockwise(current_pose_in_zero,goal2_in_zero)

                if self.index == 3:
                    robot_translation(current_pose_in_zero,goal3_in_zero)


                print(self.index)
                
                
                
                




            if self.st == State.TOP_OR_LEFT_WALL:
                # RTRT
                goal1_in_zero = [0,0,-math.pi/2]
                goal2_in_zero = [0,-1,-math.pi/2]
                goal3_in_zero = [0,-1,0]
                goal4_in_zero = [1,-1,0]

                

                if self.index == 1:
                    robot_rotation_clockwise(current_pose_in_zero,goal1_in_zero)
                    

                if self.index == 2:
                    robot_translation(current_pose_in_zero,goal2_in_zero)
                                       

                if self.index == 3:
                    robot_rotation_anticlockwise(current_pose_in_zero,goal3_in_zero)
                    

                if self.index == 4:
                    robot_translation(current_pose_in_zero,goal4_in_zero)

                print(self.index)


                    


                
                
                

    

        

    def scan_callback(self,msg):

        if self.st_wait == State.LASER_WORKING:
            """Is run whenever a LaserScan msg is received"""
            # print()
            # print("Distances:")
            # n = len(msg.ranges)
            # print("⬆️ :", msg.ranges[0])
            # print("⬇️ :", msg.ranges[n // 2])
            # print("⬅️ :", msg.ranges[n // 4])
            # print("➡️ :", msg.ranges[-n // 4])

            #print("Current state is Laser working")
            n = len(msg.ranges)
            front_distance = msg.ranges[0]
            angular_distance = msg.ranges[(11*n)// 12]
            #print(front_distance)
            #print(msg.ranges[11*n//12])

            if front_distance < 1:
                self.st = State.TOP_OR_LEFT_WALL
                print("left")

            if front_distance > 1 and angular_distance < 1:
                self.st = State.TOP_OR_LEFT_WALL
                print("top or left")
            
            if front_distance > 1 and angular_distance > 1:
                self.st = State.BOTTOM_OR_RIGHT_WALL
                print("Bottom or right")
                     
            self.st_wait = State.ODOM_WORKING

                

    
        




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