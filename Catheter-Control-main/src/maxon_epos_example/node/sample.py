import rospy
from std_msgs.msg import Float32MultiArray
from maxon_epos_msgs.msg import MotorState


class CatheterController():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.init_node("test_position_control", anonymous=True)

        self.initNode()
    
    def shutDown(self):
        rospy.loginfo("Stopping instant of EPOS...")
    

    def initNode(self):
        self.pub = rospy.Publisher("/maxon_bringup/all_position", MotorState, queue_size=1)
        


if __name__ == "__main__":

    cctrl = CatheterController()
    cctrl.pub.publish(MotorState(data=[10000, -20000]))
    
