
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
##from tello_msgs.msg import TelloAction
from tello_msgs.srv import *



class TelloScriptNode(Node):

    def __init__(self):
        super().__init__('TelloScriptNode')
        self.publisher_ = self.create_publisher(String, 'topic')
        self.cli = self.create_client(TelloAction, '/solo/tello_action')
        self.req = TelloAction.Request()
        self.future = None




        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        if(self.i % 2):
            self.doLand()
        else:
            self.doTakeoff()
        self.i += 1


    def doTakeoff(self):
        self.sendTelloCommand("takeoff")
        self.get_logger().info('takeoff')

    def doLand(self):
        self.sendTelloCommand("land")
        self.get_logger().info('land')


    def sendTelloCommand(self,cmd):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req.cmd = cmd
        self.future = 0
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    sn  = TelloScriptNode()
    sn.sendTelloCommand("battery?")


    while rclpy.ok():
        rclpy.spin_once(sn)
        if sn.future.done():
            if sn.future.result() is not None:
                response = sn.future.result()
                sn.get_logger().info('Result of drone: %d' %response.rc)
            else:
                sn.get_logger().info('Service call failed %r' % (sn.future.exception(),))

        #



    sn.destroy_node()
    rclpy.shutdown()

