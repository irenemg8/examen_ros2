from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
		
		# Definir el cliente de la accion
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose') # añadido


    def send_goal(self):

        # Incluir la posicion del goal
        # En este caso nos han pedido una sala que está en la posición x:3.25  y:8.66  z=0.0111)
        goal_msg = NavigateToPose.Goal() # añadido
        goal_msg.pose.pose.position.x = 3.25 # añadido
        goal_msg.pose.pose.position.y = 8.66 # añadido
        goal_msg.pose.pose.orientation.w = 0.0111 # añadido

        self.get_logger().info('Goal creado :)')

        self._action_client.wait_for_server()

        self.get_logger().info('AcciÃƒÂ³n activa :)')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Goal lanzado :)')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
            
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded! ')
        else:
            self.get_logger().info('Navigation failed with status: {0}'.format(status))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
