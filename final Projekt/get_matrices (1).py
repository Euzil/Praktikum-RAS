import rclpy
import numpy as np

from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped

class FrameListener(Node):

    def __init__(self):
        super().__init__('get_matrices')

        self.pose = PoseStamped()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(PoseStamped,'/VacuumGripper2/pose', self.tracking_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        
    def tracking_callback(self, msg):
        self.pose = msg.pose

    def tf_to_matrix(self, trans, rot):
        # Extract the values from Q
        print( rot )
        q0 = rot.w
        q1 = rot.x
        q2 = rot.y
        q3 = rot.z
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        # 3x3 rotation matrix
        return np.array([[r00, r01, r02, trans.x],[r10, r11, r12, trans.y],[r20, r21, r22, trans.z]])           
    
    def on_timer(self):
        try:
            # Robot matrix
            rt = self.tf_buffer.lookup_transform('base','tool0',rclpy.time.Time())
            rmat = self.tf_to_matrix(rt.transform.translation, rt.transform.rotation)
            # Tracking matrix
            tmat = self.tf_to_matrix(self.pose.position, self.pose.orientation)
            print( f'0 {rmat[0,0]:.9f} {rmat[0,1]:.9f} {rmat[0,2]:.9f} {rmat[0,3]:.9f} {rmat[1,0]:.9f} {rmat[1,1]:.9f} {rmat[1,2]:.9f} {rmat[1,3]:.9f} {rmat[2,0]:.9f} {rmat[2,1]:.9f} {rmat[2,2]:.9f} {rmat[2,3]:.9f}' )
            print( f'1 {tmat[0,0]:.9f} {tmat[0,1]:.9f} {tmat[0,2]:.9f} {tmat[0,3]:.9f} {tmat[1,0]:.9f} {tmat[1,1]:.9f} {tmat[1,2]:.9f} {tmat[1,3]:.9f} {tmat[2,0]:.9f} {tmat[2,1]:.9f} {tmat[2,2]:.9f} {tmat[2,3]:.9f}' )
        except TransformException as ex:
            self.get_logger().info('Could not transform')
            return

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()