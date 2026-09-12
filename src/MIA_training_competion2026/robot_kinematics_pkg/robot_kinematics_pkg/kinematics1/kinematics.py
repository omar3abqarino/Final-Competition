#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class omni_kinematics(Node):
    def __init__(self , name):
        super().__init__(name )
        #subscribe to encoder velocity array[v1,v2,v3,v4]
        self.subscriper = self.create_subscription(Float32MultiArray,'/encoder_velocity',self.encoder_callback,10)
        #publish to cmd_vel topic
        self.publisher = self.create_publisher(Twist, 'cmd_vel' , 10)
        self.L = 0.23 #half of the robot length in meters
        self.W =  0.15 #half of the robot width in meters
        self.R =   0.44 #wheel radius in meters
        self.get_logger().info("Omni Kinematics Node has been started")


    def encoder_callback(self,msg:Float32MultiArray ):
         if len(msg.data) != 4:
             self.get_logger().error(f"Encoder velocity array must have 4 elements , got{len(msg.data)}")
             return
         v1 , v2 , v3 , v4 = msg.data
       #calculate robot velocities using inverse kinematics
         vx ,vy , omega = self.inverse_kinematics(v1,v2,v3,v4)
            #publish to cmd_vel topic
         twist = Twist()
         twist.linear.x = vx
         twist.linear.y = vy
         twist.angular.z = omega
         self.publisher.publish(twist)
         self.get_logger().info(f"Published cmd_vel: vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}")

        
    def inverse_kinematics(self, v1, v2, v3, v4):
         raise NotImplementedError("Inverse kinematics method not implemented yet")
    


class OmniwheelKinematics(omni_kinematics):
    def __init__(self):
        super().__init__('omniwheel_kinematics')
    
    def inverse_kinematics(self, v1, v2, v3, v4):
        vx = (v1 + v2 + v3 + v4) * (self.R / 4.0)
        vy = (-v1 + v2 + v3 - v4) * (self.R / 4.0)
        omega = (-v1 + v2 - v3 + v4) * (self.R / (4.0 * (self.L + self.W)))
        return vx, vy, omega

class MecanumKinematics(omni_kinematics): 
    def __init__(self):
        super().__init__('mecanum_kinematics')
    
    def inverse_kinematics(self, v1, v2, v3, v4):
        vx = (v1 + v2 + v3 + v4) * (self.R / 4.0)
        vy = (-v1 + v2 - v3 + v4) * (self.R / 4.0)
        omega = (-v1 + v2 + v3 - v4) * (self.R / (4.0 * (self.L + self.W)))
        return vx, vy, omega   

def main(args=None):
    rclpy.init(args=args)
    # Choose which kinematics to use
    kinematics_node = OmniwheelKinematics()
    #kinematics_node = MecanumKinematics()
    rclpy.spin(kinematics_node)
    kinematics_node.destroy_node()
    rclpy.shutdown()  
if __name__ == '__main__':
    main()   