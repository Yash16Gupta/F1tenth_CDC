import rclpy
from rclpy.node import Node 
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
import numpy as np

def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('raceline')
   
    race_pub = node.create_publisher(MarkerArray, '/raceline', 10)



    path = pd.read_csv('/home/nvidia/every_10th_row.csv')
    path = path.to_numpy().reshape(-1, 2)

    print(path)
            
    for i in range(len(path)):
        path[i, 1] += -0.7
        path[i, 1] *= 0.91
        path[i, 0] += 0.3
        path[i, 0] *= 0.91 

        
    markerarray = MarkerArray()
    for i in range(len(path)):
        marker1 = Marker()
        marker1.header.frame_id = 'map'
        marker1.header.stamp = node.get_clock().now().to_msg()
        marker1.type = Marker.SPHERE
        marker1.action = Marker.ADD
        marker1.id = i
        marker1.pose.position.x = path[i][0]
        marker1.pose.position.y = path[i][1]
        marker1.pose.position.z = 0.0
        marker1.scale.x = 0.1
        marker1.scale.y = 0.1
        marker1.scale.z = 0.1
        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 1.0
        marker1.color.b = 0.0
        markerarray.markers.append(marker1)
        
    race_pub.publish(markerarray)

    rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()



if __name__=='__main__':
    main()