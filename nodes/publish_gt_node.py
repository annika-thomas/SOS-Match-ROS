#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import motlee_msgs.msg as motlee_msgs
import geometry_msgs.msg as geometry_msgs
import active_slam.config
import numpy as np

def publish_locations():
    # Initialize ROS node
    rospy.init_node('publish_gt_node')

    # Create publisher for object locations
    pub = rospy.Publisher('/gt/map', motlee_msgs.ObjArray, queue_size=5)

    # for objects in active_slam.config.gt_object_locations.yml, defined object locations as Obj
    obj_packet = motlee_msgs.ObjArray()

    Obj = motlee_msgs.Obj()
    Obj.id = 1
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 2
    x = 5.138060621793857
    y = -2.9732789020296204
    z = 0.01514832538635279
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 3
    x = 3.6810212132778424
    y = -1.483194641070801
    z = 0.15660143444872096
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 4
    x = 4.075283434692214
    y = -0.7653998965753117
    z = 0.008872907636122791
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 5
    x = 4.7274134584036345
    y = 0.8897916758060369
    z = 0.15680046102339143
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 6
    x = 3.4845039803507625
    y = 0.6337364971231915
    z = 0.0024622021276530653
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 7
    x = 2.6824525198316245
    y = 0.9430067475046596
    z = 0.21638098651564336
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 8
    x = 3.31857894427483
    y = 2.9113368244583953
    z = 0.029281356249412333
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 9
    x = 1.7450724740842913
    y = 4.185194908448791
    z = 0.578767558921031
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 10
    x = -0.1722817543658072
    y = 3.0203631101813104
    z = -0.002022159192989402
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 11
    x = -1.3519781063226017
    y = 2.7203794253784643
    z = 0.00019280324594694523
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 12
    x = -1.8708531312080925
    y = 4.2312547532906555
    z = 0.009594702260654226
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 13
    x = -3.979403390469847
    y = 2.4442921883992432
    z = 0.012107018260123408
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 14
    x = 2-4.472240952093372
    y = 1.686206901586687
    z = 0.13346038488638134
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 15
    x = -3.7652016899509735
    y = 1.4636893874727588
    z = 0.14511361803218523
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 16
    x = -3.6372154948154636
    y = -1.4501018243670147
    z = 0.22189755152361168
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 17
    x = -3.821217255755821
    y = -2.5239620998208854
    z = 0.032387993403271996
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 18
    x = -2.7690325299911858
    y = -1.9926649331074506
    z = 0.27943232630008463
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 19
    x = -1.1804150240705338
    y = -1.4673920990593143
    z = 0.020597632549389105
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 20
    x = -0.5208667780878136
    y = -1.9628814175726836
    z = 0.136030770193257
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 21
    x = 0.9878249565561752
    y = 0.44560757244005716
    z = 0.007486509451279852
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    # Publish object locations continuously
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pub.publish(obj_packet)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_locations()
    except rospy.ROSInterruptException:
        pass


    
