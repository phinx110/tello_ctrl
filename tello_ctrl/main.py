
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
        self.drone_state = state_rest()


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


######     state machine     ######

class state_rest(Object):
    def next_state(self, input):

        if(input == "takeoff"):
            return state_takeoff()
        return self

class state_takeoff(Object):
    def next_state(self, input):
        if(input == "done"):
            return state_search()
        return self

class state_search(Object):
    def next_state(self, input):
        if(input == "found"):
            return state_move()
        return self

class state_move(Object):
    def next_state(self, input):
        if(input == "Done"):
            return state_steady()
        return self

class state_steady(Object):
    def next_state(self, input):
        if(input == "OutOfPose"):
            return state_move()
        elif(input == "land"):
            return state_land()
        return self

class state_land(Object):
    def next_state(self, input):
        if(input == "done"):
            return state_rest()
        return self


def main(args=None):
    rclpy.init(args=args)
    sn  = TelloScriptNode()
    sn.sendTelloCommand("battery?")

    input = "takeoff"

    while rclpy.ok():
        rclpy.spin_once(sn)
        if sn.future.done():
            if sn.future.result() is not None:
                response = sn.future.result()
                sn.get_logger().info('Result of drone: %d' %response.rc)
            else:
                sn.get_logger().info('Service call failed %r' % (sn.future.exception(),))

        sn.drone_state = sn.drone_state.next_state(input)



    sn.destroy_node()
    rclpy.shutdown()

