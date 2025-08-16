#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mocap4r2_msgs.msg import RigidBodies
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class MocapOdometryTF(Node):
    def __init__(self):
        super().__init__('mocap_tf_broadcaster')

        # QoS 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.listener_callback,
            qos_profile
        )

        self.br = TransformBroadcaster(self)
        self.get_logger().info('MOCAP TF Broadcaster Started ')

      

    def listener_callback(self, msg: RigidBodies): 
        # pick first rigid body
        rb = msg.rigidbodies[0]
       
        # Extract MOCAP data
        x_n, y_e, z_d = rb.pose.position.x, rb.pose.position.y, rb.pose.position.z
        x_enu = x_n
        y_enu = y_e
        z_enu = z_d
        
       
        q_ned = (rb.pose.orientation.x, rb.pose.orientation.y,
                 rb.pose.orientation.z, rb.pose.orientation.w)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'px4_uav'
        t.transform.translation.x = float(x_enu)
        t.transform.translation.y = float(y_enu)
        t.transform.translation.z = float(z_enu)

        t.transform.rotation.x = float(q_ned[0])
        t.transform.rotation.y = float(q_ned[1])
        t.transform.rotation.z = float(q_ned[2])
        t.transform.rotation.w = float(q_ned[3])

        self.br.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = MocapOdometryTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
