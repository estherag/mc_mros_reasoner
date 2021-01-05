#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
MSG_DELAY = 0.2

def thruster_callback(data):
    pub_component_diagnostic(data.data)
    rospy.loginfo("thruster_callback heard %s",data.data)

def movement_callback(data):
    if data.data == "surge":
        pub_direction_diagnostic("f_nav_surge")
    elif data.data == "heave":
        pub_direction_diagnostic("f_nav_heave")
        pass
    rospy.loginfo("movement_callback heard %s",data.data)

def pub_component_diagnostic(component_name):
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = rospy.get_rostime()
    comp_msg = DiagnosticStatus()

    comp_msg.level = DiagnosticStatus.ERROR
    comp_msg.name = component_name
    comp_msg.message = "Component status"  
    diag_msg.status.append(comp_msg)

    diagnostics_pub.publish(diag_msg)
    #time.sleep(MSG_DELAY)

def pub_direction_diagnostic(function_name):
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = rospy.get_rostime()
    comp_msg = DiagnosticStatus()

    comp_msg.level = DiagnosticStatus.ERROR
    comp_msg.name = function_name
    comp_msg.message = "Movement status"
    diag_msg.status.append(comp_msg)

    diagnostics_pub.publish(diag_msg)
    #time.sleep(MSG_DELAY)

if __name__ == '__main__':
    diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
    rospy.init_node('observer', anonymous=True)
    rospy.Subscriber("/broken_thruster", String, thruster_callback)
    rospy.Subscriber("/movement", String, movement_callback)
    rospy.spin()
