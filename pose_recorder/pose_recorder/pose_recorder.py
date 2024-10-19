import rclpy
from rclpy.node import Node

from spot_recorder.srv import MyServiceMessage
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy

import yaml
from collections import OrderedDict

class RecordServer(Node):

    def __init__(self):
        super().__init__('record_spot_server')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE),
            callback_group=self.group1
        )
        
        self.service_ = self.create_service(
            MyServiceMessage,
            'record_pose',
            self.srv_callback,
            callback_group=self.group2
        )

        self.subscriber_  # prevent unused variable warning
        self.service_  # prevent unused variable warning
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.ori_x = 0.0
        self.ori_y = 0.0
        self.ori_z = 0.0
        self.ori_w = 0.0

    def pose_callback(self, msg):
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f' % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        self.get_logger().info('Recieved Data:\n X : %f \n Y : %f \n Z : %f %f' % (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w

    def srv_callback(self, request, response):

        params = {
            'pose_recorder': {
                'ros__parameters': {
                    'label': request.label,
                    'pose': {
                        'position': {
                            'x': self.pos_x,
                            'y': self.pos_y,
                            'z': self.pos_z
                        },
                        'orientation': {
                            'x': self.ori_x,
                            'y': self.ori_y,
                            'z': self.ori_z,
                            'w': self.ori_w
                        }
                    }  
                }
            }
        }

        with open("/home/user/ros2_ws/src/pose_recorder/config/spot_list.yaml", "a+") as file:
            yaml.dump(params, file, default_flow_style=False, sort_keys=False)

        
        # # TODO ADD PACKAGES PATH with rclpy func
        # with open("/home/user/ros2_ws/src/pose_recorder/config/spot_list.yaml", "a+") as f:
        #     f.write(f"\n    {request.label}:")
        #     f.write("\n       x : ")
        #     f.write(str(self.pos_x))
        #     f.write("\n       y : ")
        #     f.write(str(self.pos_y))
        #     f.write("\n       z : ")
        #     f.write(str(self.pos_z))
        #     f.write("\n       ox : ")
        #     f.write(str(self.ori_x))
        #     f.write("\n       oy : ")
        #     f.write(str(self.ori_y))
        #     f.write("\n       oz : ")
        #     f.write(str(self.ori_z))
        #     f.write("\n       ow : ")
        #     f.write(str(self.ori_w))
        

        response.navigation_successfull = True
        response.message = f"Added spot_{request.label} to spot_list.yaml"
        return response

def main(args=None):

    rclpy.init(args=args)
    record_server = RecordServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(record_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        record_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()