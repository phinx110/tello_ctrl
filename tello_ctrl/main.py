
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Char
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import TransformStamped
from tello_msgs.srv import *

from fiducial_vlam_msgs.msg import *



class TelloScriptNode(Node):

    def __init__(self):
        super().__init__('TelloScriptNode')

        self.pub_twist = self.create_publisher(Twist, '/solo/cmd_vel')

        self.cli = self.create_client(TelloAction, '/solo/tello_action')
        self.req = TelloAction.Request()

        self.future = None
        self.response = None
        self.drone_state = StateRest()

        self.key_input = None
        self.subs_key_input = self.create_subscription(Char, '/raw_keyboard', self.key_input_callback)

        self.timer_period = 0.3  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.tik = 0
        
        self.subs_observations = self.create_subscription(Observations, '/fiducial_observations', self.temp_callback)
        self.ID_1_detected = False


        
    def temp_callback(self, msg):
        for ob in msg.observations:
            if(ob.id == 1):
                self.ID_1_detected = True


    def timer_callback(self):
        self.tik = self.tik + 1
        self.drone_state = self.drone_state.next_state(self)
        self.response = None
        self.key_input = None

    def key_input_callback(self, msg):
        self.get_logger().info('received key: "%c"' % msg.data)
        self.key_input = msg.data

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
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.TelloCommand_callback)

    def TelloCommand_callback(self, future):
        if future.result() is not None:
            self.response = self.future.result().rc
            self.get_logger().info('Result of drone: %d' % self.response)
        else:
            self.get_logger().info('Service call failed %r' % (self.future.exception(),))


######     state machine     ######

class StateRest:
    def next_state(self, node):
        print(str(node.tik)+" StateRest: " + str(node.response))
        if (node.key_input == 'q') and (node.response is None):
            return StateTakeoff()
        return self

class StateTakeoff:
    def next_state(self, node):
        print(str(node.tik)+" StateTakeoff: " + str(node.response))
        if node.response is 1:
            return StateSteady()
        node.doTakeoff()
        return self

class StateLand:
    def next_state(self, node):
        print(str(node.tik)+" StateLand: " + str(node.response))
        if node.response is 1:
            return StateRest()
        node.doLand()
        return self

class StateSteady:
    def next_state(self, node):
        print(str(node.tik)+" StateSteady: " + str(node.response))
        if (node.key_input == 'q') and (node.response is None):
            node.ID_1_detected = False
            return StateLand()
        if (node.key_input == 'w') and (node.response is None):
            return StateSearchAruco(node)
        return self


class StateSearchAruco:
    
    def __init__(self, node):
        self.search_until = node.tik + 18 / node.timer_period
        #self.searching = False
        self.twist = Twist()
        self.twist.linear.z = 0.15
        self.twist.angular.z = 0.8

    def next_state(self, node):
        print("StateSearchAruco: " + str(node.response))

        if node.ID_1_detected:
            node.pub_twist.publish(Twist())
            return StateSteady()

        if node.tik <= self.search_until:
            node.pub_twist.publish(self.twist)
            return self
        node.pub_twist.publish(Twist())
        return StateSteady()


def main(args=None):
    rclpy.init(args=args)
    sn  = TelloScriptNode()
    sn.sendTelloCommand("battery?")

    rclpy.spin(sn)

    sn.destroy_node()
    rclpy.shutdown()
