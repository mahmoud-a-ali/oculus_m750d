#!/usr/bin/env python2.7

import rospy

from dynamic_reconfigure.server import Server
from oculus_sonar.cfg import OculusConfig


from rospy_message_converter import message_converter
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


# cfg_pub = rospy.Publisher("/sonar_config", Float64MultiArray, queue_size=10)
# cfg_msg = Float64MultiArray()

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {masterMode}, {gamma}, {range_m},
                  {gain}, {vOfSound}, {salinity}, {pingRate}""".format(**config))
    # print(config)
    # print(level)
    # print "configggggggggggg: "
    # print config['range_m']
    print config['groups']['parameters']

    # cfg_msg = message_converter.convert_dictionary_to_ros_message('std_msgs/Float64MultiArray', config['groups']['parameters'])


    return config

if __name__ == "__main__":
    rospy.init_node("oculus_dynamic_reconfig", anonymous = False)
    srv = Server(OculusConfig, callback)    


    # dictionary = { 'data': 'Howdy' }
    # message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', dictionary)

   # cfg_msg.data = 
   #  self.ping_pub.publish(ping_msg)
    



    rospy.spin()

