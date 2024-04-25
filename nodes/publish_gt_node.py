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
    Obj.id = 0
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 2
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 3
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 4
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 5
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 6
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 7
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 8
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 9
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 10
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 11
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 12
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 13
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 14
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 15
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 16
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 17
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 18
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 19
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 20
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
    Obj.position = geometry_msgs.Point(x=x, y=y, z=z)
    obj_packet.objects.append(Obj)

    Obj = motlee_msgs.Obj()
    Obj.id = 21
    x = 2.8209561647284094
    y = -2.799555938150988
    z = 0.022653042195593026
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


    
