import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")
        self.sub_=self.create_subscription(
            String,"chatter",self.msgCallBack,10
        )



    def msgCallBack(self,msg):
        self.get_logger().info(f"i heared {msg.data}")

def main():
    rclpy.init()
    simple_subscriber=SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()         
