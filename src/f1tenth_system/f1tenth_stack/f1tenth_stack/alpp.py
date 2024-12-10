import numpy as np
import rclpy
import rclpy.logging
import tf_transformations
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from math import atan2, sqrt, pow, sin, exp
import pandas as pd
import time
from ackermann_msgs.msg import AckermannDriveStamped

class AdaptivePurePursuit(Node):
    def __init__(self):
        super().__init__('adaptive_pure_pursuit')

        self.load_raceline_csv(
            '/home/nvidia/Downloads/sahil_mc_raceline.csv')
        self.pos_sub = self.create_subscription(
            Odometry, '/odom', self.pos_callback, 10)
        self.thr_pub = self.create_publisher(
            Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.str_pub = self.create_publisher(
            Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal', 10)
        self.cp_pub = self.create_publisher(Marker, '/cp', 10)
        self.race_pub = self.create_publisher(MarkerArray, '/raceline', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        timer = 0.0001       
        self.timer = self.create_timer(timer, self.publish_control_commands)
        self.max_speed = 3.0
        self.min_speed = 2.0
        self.max_lookahead = 2.8
        self.min_lookahead = 1.7
        self.wheelbase = 0.33
        self.current_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.lookahead_distance = self.min_lookahead
        self.beta = 0.5
        # self.path = np.array([])
        self.previous_position = None
        self.previous_deviation = 0
        self.total_area = 0
        self.area_window = []
        self.window_size = 10
        self.position = None
        self.orientation = None
        self.control_velocity = 0.0015
        self.heading_angle = 0.01
        self.weighted_heading = 0.01
        self.heading_scale = 0.8
        self.area_threshold = 1.0
        self.speed_factor = 0.4
        self.velocity_superintendence_1 = 2
        self.velocity_superintendence_2 = 0.8
        self.velocity_window = []
        self.window_velocity_size = 10
        self.smoothed_velocity = 2.0
        self.previous_goal=[0,0]
        self.s = 0.0
        self.decay = 0.8



        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # self.load_raceline_csv(
        #     '/home/nvidia/rcl/center_line.csv')
        # self.get_logger().info('construction done')

    def load_raceline_csv(self, filename):
        self.path = pd.read_csv(filename)
        self.path = np.array([self.path]).reshape(-1, 2)
        self.path = self.path[::60]
        # print("before", self.path.shape)
        self.path = self.path[::-1]
        self.path = np.vstack((self.path, self.path[:10] ))
        # print("after", self.path.shape)


        # self.df = pd.read_csv(filename)
        # self.path = np.column_stack((self.df['x'], self.df["y"]))
        # self.path = self.path[::50]
        # self.path = self.path[::-1]

       # for i in range(len(self.path)):
        #     self.path[i, 1] += 1.0
         #    self.path[i, 1] *= 1
          #   self.path[i, 0] -= 1.5
           #  self.path[i, 0] *= 1

        # rotation_matrix = np.array([[-0.06, 1], [-1, 0]])
        # self.path = np.dot(self.path, rotation_matrix.T)

    def sigmoid(self, x):
        return 1 / (1 + exp(-x))

    def update_lookahead(self, speed):
        normalized_speed = (speed - self.min_speed) / \
            (self.max_speed - self.min_speed)
        sigmoid_value = self.sigmoid(normalized_speed * 10 - 5)

        if speed < self.min_speed:
            self.lookahead_distance = self.min_lookahead
        else:
            scaled_lookahead = self.min_lookahead + sigmoid_value * \
                (self.max_lookahead - self.min_lookahead)
            self.lookahead_distance = min(self.max_lookahead, scaled_lookahead)

    def pos_callback(self, msg):
        try:
            time1 = time.time()
        
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=10.0))
            
            #transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            #transform2 = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())

            # Convert TransformStamped to tf2's Transform
            #map_to_odom = tf_transformations.concatenate_matrices(
             #   tf_transformations.translation_matrix((
              #      transform.transform.translation.x,
               #     transform.transform.translation.y,
                    #transform.transform.translation.z
               # )),
                #tf_transformations.quaternion_matrix((
                 #   transform.transform.rotation.x,
         #           transform.transform.rotation.y,
        #            transform.transform.rotation.z,
              #      transform.transform.rotation.w
             #   ))
            #)
            
          #  odom_to_base_link = tf_transformations.concatenate_matrices(
           #     tf_transformations.translation_matrix((
            #        transform2.transform.translation.x,
             #       transform2.transform.translation.y,
              #      transform2.transform.translation.z
           #     )),
            #    tf_transformations.quaternion_matrix((
             #       transform2.transform.rotation.x,
                #    transform2.transform.rotation.y,
               #     transform2.transform.rotation.z,
              #      transform2.transform.rotation.w
             #   ))
            #)
            
            # Combine the two transforms
            #map_to_base_link = tf_transformations.concatenate_matrices(map_to_odom, odom_to_base_link)
            
            # Extract the resulting translation and rotation
            #translation = tf_transformations.translation_from_matrix(map_to_base_link)
            #rotation = tf_transformations.quaternion_from_matrix(map_to_base_link)
            
            # Create a TransformStamped for the result
            #transform_result = TransformStamped()
            #transform_result.header.stamp = transform.header.stamp
            #transform_result.header.frame_id = "map"
            #transform_result.child_frame_id = "base_link"
            #transform_result.transform.translation.x = translation[0]
            #transform_result.transform.translation.y = translation[1]
            #transform_result.transform.translation.z = translation[2]
            #transform_result.transform.rotation.x = rotation[0]
            #transform_result.transform.rotation.y = rotation[1]
            ##transform_result.transform.rotation.z = rotation[2]
            #transform_result.transform.rotation.w = rotation[3]
            
            # Set the position and orientation attributes
            #self.position = transform_result.transform.translation
            #self.orientation = transform_result.transform.rotation
            self.position = transform.transform.translation
            self.orientation = transform.transform.rotation
            print("position" , self.position, "orientation", self.orientation)
            self.current_quaternion = [
                self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
            self.yaw = self.quaternion_to_yaw(self.current_quaternion)
            print("yaw", self.yaw)

            current_speed = msg.twist.twist.linear.x

            self.update_lookahead(current_speed)
            # print('2')
            closest_point, goal_point = self.get_lookahead_point(self.position)
            # print('3')
            
            if goal_point is not None:
                self.previous_goal = goal_point
                alpha = self.calculate_alpha(self.position, goal_point, self.yaw)
                self.heading_angle = self.calculate_heading_angle(alpha)
                self.weighted_heading = self.moving_average_filter(self.heading_angle)

                area = self.calculate_deviation(self.position, closest_point)

                max_velocity_pp = self.calculate_max_velocity_pure_pursuit(
                    self.calculate_curvature(alpha))
                min_deviation_pp = self.calculate_min_deviation_pure_pursuit(area)

                self.control_velocity = self.convex_combination(
                    max_velocity_pp, min_deviation_pp, current_speed, area)
                self.smoothed_velocity = self.moving_average_filter(self.control_velocity)
                self.publish_control_commands()
                time2 = time.time()
                time_final = time2-time1
                #print(time_final) 
            else:
                goal_point = self.previous_goal

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform lookuppppppp failed: {e}") 
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Transform exception failed: {e}")

    def quaternion_to_yaw(self, quaternion):
        qx, qy, qz, qw = quaternion
        siny_cosp = 2*(qw * qz + qx * qy)
        cosy_cosp = 1 - 2*(qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)

        return yaw

    def get_lookahead_point(self, position):
        min_dist = float('inf')
        closest_point = None
        goal_point = None
        for point in self.path[0:len(self.path) - 10]:
            dist = sqrt(pow(point[0] - position.x, 2) +
                        pow(point[1] - position.y, 2))
            # print(point)
            if dist < min_dist:
                min_dist = dist
                closest_point = point
        closest_point_index = np.where(self.path == closest_point)

        for point in self.path[0:len(self.path) -10 ]:
            dist = sqrt(pow(point[0] - position.x, 2) +
                        pow(point[1] - position.y, 2))
            point_index = np.where(self.path == point)
            #do not change the line below at any cost, warna yash bhaiya gand tod denge.   
            if dist > self.lookahead_distance and point_index[0][0] > closest_point_index[0][0] + 1  and point_index[0][0] < closest_point_index[0][0] + 10:
                #print("In loop")
                # print ("Dummy",self.min_lookahead, self.max_lookahead)
                goal_point = point
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()

                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker2 = Marker()
                marker2.header.frame_id = 'map'
                marker2.header.stamp = self.get_clock().now().to_msg()

                marker2.type = Marker.SPHERE
                marker2.action = Marker.ADD
                marker2.pose.position.x = closest_point[0]
                marker2.pose.position.y = closest_point[1]
                marker2.pose.position.z = 0.0

                marker2.scale.x = 0.1
                marker2.scale.y = 0.1
                marker2.scale.z = 0.1

                marker2.color.a = 1.0
                marker2.color.r = 0.0
                marker2.color.g = 0.0
                marker2.color.b = 1.0

                markerarray = MarkerArray()
                for i in range(len(self.path)):
                    marker1 = Marker()
                    marker1.header.frame_id = 'map'
                    marker1.header.stamp = self.get_clock().now().to_msg()

                    marker1.type = Marker.SPHERE
                    marker1.action = Marker.ADD
                    marker1.id = i
                    marker1.pose.position.x = self.path[i][0]
                    marker1.pose.position.y = self.path[i][1]
                    marker1.pose.position.z = 0.0

                    marker1.scale.x = 0.1
                    marker1.scale.y = 0.1
                    marker1.scale.z = 0.1

                    marker1.color.a = 1.0
                    marker1.color.r = 0.0
                    marker1.color.g = 1.0
                    marker1.color.b = 0.0
                    markerarray.markers.append(marker1)

                self.goal_pub.publish(marker)
                self.cp_pub.publish(marker2)
                self.race_pub.publish(markerarray)
                break
        return closest_point, goal_point

    def calculate_alpha(self, position, goal_point, yaw):
        dy = goal_point[1] - position.y
        dx = goal_point[0] - position.x
        local_x = dx*np.cos(-yaw) - dy*np.sin(-yaw)
        local_y = dx*np.sin(-yaw) + dy*np.cos(-yaw)
        alpha = atan2(local_y, local_x)
        return alpha

    def calculate_heading_angle(self, alpha):
        heading_angle = atan2(2 * self.wheelbase *
                              sin(alpha), self.lookahead_distance)
        return heading_angle

    def calculate_curvature(self, alpha):
        curvature = 2 * sin(alpha) / self.lookahead_distance
        return curvature

    def calculate_deviation(self, position, closest_point):
        deviation = sqrt(
            pow(closest_point[0] - position.x, 2) + pow(closest_point[1] - position.y, 2))

        if self.previous_position is not None:
            distance_traveled = sqrt(pow(position.x - self.previous_position.x, 2) +
                                     pow(position.y - self.previous_position.y, 2))
            area_increment = (
                deviation + self.previous_deviation) / 2 * distance_traveled

            self.area_window.append(area_increment)
            if len(self.area_window) > self.window_size:
                self.area_window.pop(0)

            self.total_area = sum(self.area_window)

        self.previous_position = position
        self.previous_deviation = deviation

        return self.total_area

    def calculate_max_velocity_pure_pursuit(self, curvature):
        max_velocity = sqrt(
            1 / abs(curvature)) if curvature != 0 else self.max_speed
        return min(self.max_speed, max_velocity)

    def calculate_min_deviation_pure_pursuit(self, area):
        if area > 0:
            min_deviation_velocity = self.max_speed / (1 + area)
        else:
            min_deviation_velocity = self.max_speed
        return min_deviation_velocity

    def convex_combination(self, max_velocity_pp, min_deviation_pp, current_speed, area):
        self.beta = self.adjust_beta(current_speed, area)
        control_velocity = self.beta * max_velocity_pp + \
            (1 - self.beta) * min_deviation_pp
        # curvature = self.calculate_curvature(self.heading_angle)
        # curv_diff = abs(curvature)
        # control_velocity /= exp(self.velocity_superintendence_1 * (abs(curv_diff) ** self.velocity_superintendence_2))
        return control_velocity

    def adjust_beta(self, current_speed, area):
        if area < self.area_threshold:
            return min(1.0, self.beta + 0.25)
        elif current_speed < self.max_speed * self.speed_factor:
            return max(0.0, self.beta - 0.25)
        return self.beta
    
    def moving_average_filter(self, velocity):
        self.velocity_window.append(velocity)
        if len(self.velocity_window) > self.window_velocity_size:
            self.velocity_window.pop(0)
            self.s=0.0
        for i in range(len(self.velocity_window)):
            self.s += self.velocity_window[i] * (self.decay ** (len(self.velocity_window) - 1 - i))

        return self.s/len(self.velocity_window)

    def publish_control_commands(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = 'map'
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = self.control_velocity
        drive_msg.drive.steering_angle = max(-0.14, min(self.heading_angle * self.heading_scale , 0.14))
        print("steer:", np.rad2deg(self.heading_angle*self.heading_scale), "velocity:", self.smoothed_velocity)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)

    adaptive_pure_pursuit = AdaptivePurePursuit()
    rate = adaptive_pure_pursuit.create_rate(0.1)
    try:
        rclpy.spin(adaptive_pure_pursuit)
        rate.sleep()
    except KeyboardInterrupt:
        pass
    adaptive_pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
