#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import motlee_msgs.msg as motlee_msgs
import geometry_msgs.msg as geometry_msgs
import active_slam.config
import numpy as np
from visualization_msgs.msg import Marker

def make_marker(x, y, z, id):

    marker = Marker()
    #marker.header.frame_id = odom_msg.header.frame_id
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    # marker.pose.orientation.x = objmsg.pose.orientation.x
    # marker.pose.orientation.y = objmsg.pose.orientation.y
    # marker.pose.orientation.z = objmsg.pose.orientation.z
    # marker.pose.orientation.w = objmsg.pose.orientation.w
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.scale.x = 0.2  # Point size
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.r = 1.0  # Color (red)
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.header.frame_id = "world"


    print(f"Object at: {x}, {y}, {z}")

    # Publish the marker
    return marker

def publish_locations():
    # Initialize ROS node
    rospy.init_node('publish_gt_node')

    # Create publisher for object locations
    pub = rospy.Publisher('/gt/map', motlee_msgs.ObjArray, queue_size=5)
    marker_pub = rospy.Publisher('/gt/marker', Marker, queue_size=5)

    # for objects in active_slam.config.gt_object_locations.yml, defined object locations as Obj
    obj_packet = motlee_msgs.ObjArray()

    # Obj = motlee_msgs.Obj()
    # Obj.id = 1
    # x = 2.8209561647284094
    # y = -2.799555938150988
    # z = 0.022653042195593026
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker1 = make_marker(x, y, z, 1)


    # Obj = motlee_msgs.Obj()
    # Obj.id = 2
    # x = 5.138060621793857
    # y = -2.9732789020296204
    # z = 0.01514832538635279
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker2 = make_marker(x, y, z, 2)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 3
    # x = 3.6810212132778424
    # y = -1.483194641070801
    # z = 0.15660143444872096
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker3 = make_marker(x, y, z, 3)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 4
    # x = 4.075283434692214
    # y = -0.7653998965753117
    # z = 0.008872907636122791
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker4 = make_marker(x, y, z, 4)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 5
    # x = 4.7274134584036345
    # y = 0.8897916758060369
    # z = 0.15680046102339143
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker5 = make_marker(x, y, z, 5)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 6
    # x = 3.4845039803507625
    # y = 0.6337364971231915
    # z = 0.0024622021276530653
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker6 = make_marker(x, y, z, 6)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 7
    # x = 2.6824525198316245
    # y = 0.9430067475046596
    # z = 0.21638098651564336
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker7 = make_marker(x, y, z, 7)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 8
    # x = 3.31857894427483
    # y = 2.9113368244583953
    # z = 0.029281356249412333
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker8 = make_marker(x, y, z, 8)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 9
    # x = 1.7450724740842913
    # y = 4.185194908448791
    # z = 0.578767558921031
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker9 = make_marker(x, y, z, 9)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 10
    # x = -0.1722817543658072
    # y = 3.0203631101813104
    # z = -0.002022159192989402
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker10 = make_marker(x, y, z, 10)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 11
    # x = -1.3519781063226017
    # y = 2.7203794253784643
    # z = 0.00019280324594694523
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker11 = make_marker(x, y, z, 11)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 12
    # x = -1.8708531312080925
    # y = 4.2312547532906555
    # z = 0.009594702260654226
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker12 = make_marker(x, y, z, 12)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 13
    # x = -3.979403390469847
    # y = 2.4442921883992432
    # z = 0.012107018260123408
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker13 = make_marker(x, y, z, 13)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 14
    # x = 2-4.472240952093372
    # y = 1.686206901586687
    # z = 0.13346038488638134
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker14 = make_marker(x, y, z, 14)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 15
    # x = -3.7652016899509735
    # y = 1.4636893874727588
    # z = 0.14511361803218523
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker15 = make_marker(x, y, z, 15)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 16
    # x = -3.6372154948154636
    # y = -1.4501018243670147
    # z = 0.22189755152361168
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker16 = make_marker(x, y, z, 16)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 17
    # x = -3.821217255755821
    # y = -2.5239620998208854
    # z = 0.032387993403271996
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker17 = make_marker(x, y, z, 17)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 18
    # x = -2.7690325299911858
    # y = -1.9926649331074506
    # z = 0.27943232630008463
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker18 = make_marker(x, y, z, 18)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 19
    # x = -1.1804150240705338
    # y = -1.4673920990593143
    # z = 0.020597632549389105
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker19 = make_marker(x, y, z, 19)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 20
    # x = -0.5208667780878136
    # y = -1.9628814175726836
    # z = 0.136030770193257
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker20 = make_marker(x, y, z, 20)

    # Obj = motlee_msgs.Obj()
    # Obj.id = 21
    # x = 0.9878249565561752
    # y = 0.44560757244005716
    # z = 0.007486509451279852
    # Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    # obj_packet.objects.append(Obj)
    # marker21 = make_marker(x, y, z, 21)
    
    # CORRECTED GT FOR DRONE MAP
    
    Obj = motlee_msgs.Obj()
    Obj.id = 1
    x = 2.81
    y = -2.73
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker1 = make_marker(x, y, z, 1)


    Obj = motlee_msgs.Obj()
    Obj.id = 2
    x = 5.16
    y = -2.96
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker2 = make_marker(x, y, z, 2)

    Obj = motlee_msgs.Obj()
    Obj.id = 3
    x = 3.55
    y = -1.28
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker3 = make_marker(x, y, z, 3)

    Obj = motlee_msgs.Obj()
    Obj.id = 4
    x = 4.07
    y = -0.76
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker4 = make_marker(x, y, z, 4)

    Obj = motlee_msgs.Obj()
    Obj.id = 5
    x = 3.49
    y = 0.64
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker5 = make_marker(x, y, z, 5)

    Obj = motlee_msgs.Obj()
    Obj.id = 6
    x = 4.71
    y = 0.98
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker6 = make_marker(x, y, z, 6)

    Obj = motlee_msgs.Obj()
    Obj.id = 7
    x = 2.39
    y = 1.16
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker7 = make_marker(x, y, z, 7)

    Obj = motlee_msgs.Obj()
    Obj.id = 8
    x = 3.32
    y = 2.91
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker8 = make_marker(x, y, z, 8)

    Obj = motlee_msgs.Obj()
    Obj.id = 9
    x = 1.03
    y = 0.48
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker9 = make_marker(x, y, z, 9)

    Obj = motlee_msgs.Obj()
    Obj.id = 10
    x = -0.12
    y = 3.00
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker10 = make_marker(x, y, z, 10)

    Obj = motlee_msgs.Obj()
    Obj.id = 11
    x = -1.34
    y = 2.70
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker11 = make_marker(x, y, z, 11)

    Obj = motlee_msgs.Obj()
    Obj.id = 12
    x = -1.87
    y = 4.19
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker12 = make_marker(x, y, z, 12)

    Obj = motlee_msgs.Obj()
    Obj.id = 13
    x = -4.00
    y = 2.45
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker13 = make_marker(x, y, z, 13)

    Obj = motlee_msgs.Obj()
    Obj.id = 14
    x = -4.43
    y = 1.70
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker14 = make_marker(x, y, z, 14)

    Obj = motlee_msgs.Obj()
    Obj.id = 15
    x = -3.75
    y = 1.47
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker15 = make_marker(x, y, z, 15)

    Obj = motlee_msgs.Obj()
    Obj.id = 16
    x = -3.18
    y = -1.52
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker16 = make_marker(x, y, z, 16)

    Obj = motlee_msgs.Obj()
    Obj.id = 17
    x = -3.85
    y = -2.56
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker17 = make_marker(x, y, z, 17)

    Obj = motlee_msgs.Obj()
    Obj.id = 18
    x = -2.22
    y = -2.39
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker18 = make_marker(x, y, z, 18)

    Obj = motlee_msgs.Obj()
    Obj.id = 19
    x = -1.13
    y = -1.48
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker19 = make_marker(x, y, z, 19)

    Obj = motlee_msgs.Obj()
    Obj.id = 20
    x = -0.55
    y = -1.87
    z = 0
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)
    marker20 = make_marker(x, y, z, 20)

    

    # Publish object locations continuously
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pub.publish(obj_packet)
        marker_pub.publish(marker1)
        marker_pub.publish(marker2)
        marker_pub.publish(marker3)
        marker_pub.publish(marker4)
        marker_pub.publish(marker5)
        marker_pub.publish(marker6)
        marker_pub.publish(marker7)
        marker_pub.publish(marker8)
        marker_pub.publish(marker9)
        marker_pub.publish(marker10)
        marker_pub.publish(marker11)
        marker_pub.publish(marker12)
        marker_pub.publish(marker13)
        marker_pub.publish(marker14)
        marker_pub.publish(marker15)
        marker_pub.publish(marker16)
        marker_pub.publish(marker17)
        marker_pub.publish(marker18)
        marker_pub.publish(marker19)
        marker_pub.publish(marker20)
        #marker_pub.publish(marker21)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_locations()
    except rospy.ROSInterruptException:
        pass


    
