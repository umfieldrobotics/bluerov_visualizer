#!/bin/python3

# Script to visualize everything related to the acoustics, including the map, DVL beams, ping sonar and (soon) ping_360

import rospy
from sensor_msgs.msg import Range
from waterlinked_a50_ros_driver.msg import DVLBeam, DVL
from ping_sonar.msg import SonarEcho, SonarEchoProfile


def ping_callback(msg: SonarEcho):
    publisher = rospy.Publisher('/ping_sonar/range_viz', Range, queue_size=10)
    range_msg = Range()
    range_msg.header = msg.header
    range_msg.header.frame_id = 'sonar_link_viz'
    range_msg.range = msg.distance
    range_msg.min_range = 0.3
    range_msg.max_range = 100.0
    range_msg.field_of_view = 0.436334
    range_msg.radiation_type = 0
    publisher.publish(range_msg)

def dvl_range_callback(msg: DVL):
    publisher1 = rospy.Publisher('/dvl/range1_viz', Range, queue_size=10)
    publisher2 = rospy.Publisher('/dvl/range2_viz', Range, queue_size=10)
    publisher3 = rospy.Publisher('/dvl/range3_viz', Range, queue_size=10)
    publisher4 = rospy.Publisher('/dvl/range4_viz', Range, queue_size=10)

    range_msg1 = Range()
    range_msg2 = Range()
    range_msg3 = Range()
    range_msg4 = Range()
    
    beam1:DVLBeam = msg.beams[0]
    range_msg1.header = msg.header
    range_msg1.header.frame_id = 'transducer1_link_viz'
    range_msg1.range = beam1.distance
    range_msg1.min_range = 0.05
    range_msg1.max_range = 35.0
    range_msg1.field_of_view = 0.0767945
    range_msg1.radiation_type = 0
    publisher1.publish(range_msg1)

    beam2:DVLBeam = msg.beams[1]
    range_msg2.header = msg.header
    range_msg2.header.frame_id = 'transducer2_link_viz'
    range_msg2.range = beam2.distance
    range_msg2.min_range = 0.05
    range_msg2.max_range = 35.0
    range_msg2.field_of_view = 0.0767945
    range_msg2.radiation_type = 0
    publisher2.publish(range_msg2)
    
    beam3:DVLBeam = msg.beams[2]
    range_msg3.header = msg.header
    range_msg3.header.frame_id = 'transducer3_link_viz'
    range_msg3.range = beam3.distance
    range_msg3.min_range = 0.05
    range_msg3.max_range = 35.0
    range_msg3.field_of_view = 0.0767945
    range_msg3.radiation_type = 0
    publisher3.publish(range_msg3)

    beam4:DVLBeam = msg.beams[3]
    range_msg4.header = msg.header
    range_msg4.header.frame_id = 'transducer4_link_viz'
    range_msg4.range = beam4.distance
    range_msg4.min_range = 0.05
    range_msg4.max_range = 35.0
    range_msg4.field_of_view = 0.0767945
    range_msg4.radiation_type = 0
    publisher4.publish(range_msg4)



def main():
    rospy.init_node('acoustics_visualization')
    rospy.Subscriber('/ping_sonar/data', SonarEcho, ping_callback)
    rospy.Subscriber('/dvl/data', DVL, dvl_range_callback)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()