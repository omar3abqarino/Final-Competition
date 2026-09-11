from rclpy.node import Node
import rclpy, math, time
from rclpy.action import ActionServer
from custom_interfaces.action import Move
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



class MoveX_Server(Node):
    def __init__(self):
        super().__init__("action_x_server")

        # To Prevent Intersecting Threads
        self.cb_group = ReentrantCallbackGroup()


        
        self.action_server = ActionServer(self, Move, "move_robot_x", execute_callback = self.execute_callback, callback_group = self.cb_group)
        
        self.odom_subscriber = self.create_subscription(Odometry, "/odom", self.odom_callback, 10, callback_group = self.cb_group)


        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.position = None
        self.last_odom_time = 0.0

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.last_odom_time = time.time()



    def execute_callback(self, goal):
    
        result = Move.Result()
        feedback = Move.Feedback()

        start_time = time.time()

        # Odometry Data Missing (Edge Case 1)
        while self.position is None:
            if time.time() - start_time > 5.0:
                self.get_logger().info("No Odometry Data Recieved")
                goal.abort()
                result.success = False
                result.message = "Odometry Missing"
                return result
            time.sleep(0.1)


        target = goal.request.target_x

        initial_x = self.position.x
        initial_y = self.position.y

        # Forward and Backward Movement
        if target > 0:
            speed = 0.2
            forward = True
        elif target < 0:
            speed = 0.2
            forward = False
        else:
            result.success = True
            result.message = "No Movement Happened"
            return result
    
        msg = Twist()


        if forward:
            msg.linear.x = float(speed)
        else:
            msg.linear.x = -float(speed)

        msg.angular.z = 0.0



        max_duration = float((abs(target) / speed) + 5)
        motion_start_time = time.time()


        while rclpy.ok():

            #Timeout (Edge Case 2)
            if time.time() - motion_start_time > max_duration:
                self.stop_robot()
                goal.abort()
                result.success = False
                result.message = "Timeout"
                return result


            
            dx = self.position.x - initial_x
            dy = self.position.y - initial_y
            dist_done = math.sqrt(dx**2 + dy**2)

            
            feedback.progress = float(dist_done / abs(target))*100
            goal.publish_feedback(feedback)

            # Target Reached
            if dist_done >= abs(target):
                break

            self.publisher.publish(msg)

            time.sleep(0.1)


        self.stop_robot()
        goal.succeed()
        result.success = True
        result.message = "Target Reached"
        return result

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def main():

    rclpy.init()
    node = MoveX_Server()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()