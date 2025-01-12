import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

LENGTH = 0.281
WIDTH =  0.306
PADDING = 0.05
MIN_DIST_FROM_WALL = LENGTH/2 + PADDING

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.window = 4
        self.front_dists = [0.0]*self.window
        self.front_dist_list = []
        self.front_dist_avg_list = []
        self.front_dist_avg = 0.0
        self.front_dist_sum = 0.0
        self.index = 0
        self.velocity = 0.0
        self.velos_list = []
        

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
                        
        # print()
        # print('Distances:')
        # print('⬆️ :', msg.ranges[0])
        # print('⬇️ :', msg.ranges[180])
        # print('⬅️ :', msg.ranges[90])
        # print('➡️ :', msg.ranges[-90])

      
        self.front_dist_list.append(msg.ranges[0])
        self.front_dist_sum = self.front_dist_sum - self.front_dists[self.index]
        self.front_dists[self.index] = msg.ranges[0]
        self.front_dist_sum = self.front_dist_sum + msg.ranges[0]
        self.index = (self.index + 1) % self.window


        # For values before the first window as the avg is smaller, we are considering the original values until first window
        if len(self.front_dist_avg_list) < self.window:
            self.front_dist_avg = msg.ranges[0]

        else:
            self.front_dist_avg = self.front_dist_sum/self.window
        
        self.front_dist_avg_list.append(self.front_dist_avg)

        



        print()
        print('Distances:')
        print('⬆️ :', self.front_dist_avg)
        print('⬇️ :', msg.ranges[180])
        print('⬅️ :', msg.ranges[90])
        print('➡️ :', msg.ranges[-90])
                
        

        

        acceleration_factor = 0.01
        deceleration_factor = -0.01
        delta_t = 0.2
        max_vel = 0.03 # user sets the max velocity in meters per second
        #max_vel_ms = max_vel*0.26/100 # converting into percentage 
        distance_to_decelerate = (max_vel*max_vel)/(2*(-deceleration_factor))
        
        
        if self.front_dist_avg > distance_to_decelerate + MIN_DIST_FROM_WALL:
            if self.velocity < max_vel:
                self.velocity = self.velocity + acceleration_factor*delta_t
            else:
                pass

            self.velos_list.append(self.velocity)
            self.vel(self.velocity*100/0.26)

        else:                     
            if self.front_dist_avg > MIN_DIST_FROM_WALL:                
                self.velocity = self.velocity + deceleration_factor*delta_t
                self.velos_list.append(self.velocity)
                self.vel(self.velocity*100/0.26)
                print("Velocity: %1.3f"%self.velocity)
            else:
                self.vel(0)
                

            
                    




def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        plt.plot(tb3.front_dist_list, label = "Actual")      
        plt.plot(tb3.front_dist_avg_list,label = "Average")
        plt.title("Actual Data Vs Averaged data with window size %i"%tb3.window)
        plt.legend()
        plt.show()

        plt.plot(tb3.velos_list)
        plt.title("Velocity graph")
        plt.show()
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
