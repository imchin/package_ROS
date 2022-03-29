from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
 

class MinimalService(Node):

    def __init__(self):
        super().__init__('t_service')
        self.srv = self.create_service(SetBool,'qwer',self.t_service_callback)
        

    def t_service_callback(self, request, response):
        self.get_logger().info("In : " + str(request.data))
        response.message = "tee kai"
        return response


def main(args=None):
    rclpy.init(args=args)
    tservice = MinimalService()
    rclpy.spin(tservice)

    tservice.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()