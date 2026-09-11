from platform import node

import rclpy, time
from rclpy.node import Node
from rclpy.action import ActionClient
import math
from std_srvs.srv import SetBool

from custom_interfaces.action import Move


class Action_Node(Node):
    def __init__(self):
        super().__init__('Action_Node')

        #creating action client 
        self.yaw_client = ActionClient(self,Move,'/move_yaw')
        self.move_client = ActionClient(self, Move, 'move_robot_x')
        self.wall_client = self.create_client(SetBool,'/toggle_walls_1_2') #create service client
        

    def solve_maze(self):
        
        time.sleep(1)
        self.send_yaw_goal(80)
        time.sleep(6)
        self.send_request(True)
        time.sleep(6)
        self.send_move_goal(1.0)
        time.sleep(6)
        self.send_move_goal(1.0)
        time.sleep(6)
        self.send_request(False)
        time.sleep(6)
        self.send_move_goal(0.8)
        time.sleep(6)
        self.send_yaw_goal(-30)
        time.sleep(6)

        
       
       
        for _ in range(7):
            self.send_move_goal(1.0)
            time.sleep(6)
        


    def send_yaw_goal(self,target):
        #waiting for server to start
        self.get_logger().info("waiting for yaw server...")
        self.yaw_client.wait_for_server(timeout_sec = 1.5)
        #make the goal msg
        gms = Move.Goal()
        gms.target_yaw = math.radians(target)
        gms.forward_distance = 0.0
    
        # send goal
        self.send_goal_future = self.yaw_client.send_goal_async(gms,feedback_callback = self.yaw_feedback)
        # check if server accepted
        self.send_goal_future.add_done_callback(self.yaw_goal_callback)
    def yaw_feedback(self , fmsg):
        # get the feedback msg
        f = fmsg.feedback
        # print it 
        self.get_logger().info(f'feedback = current action = {f.current_action}  progress = {f.progress}')
    def yaw_goal_callback(self ,future):
        # got result 
        goal_handle = future.result()
        if not goal_handle.accepted:
            # server didnt accept
            self.get_logger().info("not accepted")
            return
        else:
            self.get_logger().info('accepted')
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.yaw_result_callback)
            return 
    def yaw_result_callback(self ,future):
        result  = future.result().result
        if result.success:
            self.get_logger().info(f"success {result.message}")
        else:
            self.get_logger().error(result.message)    



    def send_move_goal(self,move):

        # Wait for the action server to be available
        self.get_logger().info("Waiting for movement server......")
        self.move_client.wait_for_server(timeout_sec=1.5)

        # Create a goal message
        goal_msg = Move.Goal()
        goal_msg.target_yaw = 0.0
        goal_msg.target_x = move 
        
        # Send the goal to the action server
        self.send_goal_future = self.move_client.send_goal_async(goal_msg, feedback_callback=self.move_feedback)
        self.send_goal_future.add_done_callback(self.move_goal_callback)


    def move_goal_callback(self, future):

        # Check if the goal was accepted by the server
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by server.")
            return
        
        self.get_logger().info("Goal accepted by the server.")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.move_result_callback)


    def move_feedback(self, feedback_msg):

        # Handle feedback from the action server
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback received: Current action = {feedback.current_action}, Progress = {feedback.progress}")


    def move_result_callback(self, future):

        # Handle the result from the action server
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Action completed successfully: {result.message}")
        else:
            self.get_logger().error(f"Action failed: {result.message}")




    def send_request(self,var):
        if not self.wall_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error("service not available") #checks if the service is available
            return
        request= SetBool.Request() #sends request to service
        request.data = var
        future = self.wall_client.call_async(request)
        future.add_done_callback(
            self.service_response
        )

    def service_response(self,future):
        response = future.result()
        self.get_logger().info( f'success={response.success}') #prints the case
        self.get_logger().info( f'message={response.message}') # prints a message of the walls



    
def main():
    rclpy.init()
    node = Action_Node()
    node.solve_maze()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()