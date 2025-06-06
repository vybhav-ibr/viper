import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from cv_bridge import CvBridge
import cv2

class RosDroidSim(Node):
    def __init__(self,exp_name):
        super().__init__('droid_env_node')

        # Class member variables to store latest messages
        self.left_ext_cam_img= None
        self.left_wrist_cam_img = None
        self.viewer_cam_img =None
        self.prompt = None
        self.joint_states = None
        
        self.bridge = CvBridge()
        self.video_writer = cv2.VideoWriter(
            f'../videos/{exp_name}.mp4',
            cv2.VideoWriter_fourcc(*'mp4v'),
            5,
            (640,480)
        )
        
        self.joints={
             "arm": [f'joint{index}' for index in range(7)],
             "gripper":["left_finger","right_finger"]
        }

        # Camera subscribers (using lambda)
        self.create_subscription(Image, '/viewer_cam/image_raw', self.viewer_callback, 5)
        self.create_subscription(Image, '/left_ext_cam/image_raw', lambda msg: setattr(self, 'left_ext_cam_img', msg), 30)
        self.create_subscription(Image, '/left_wrist_cam/image_raw', lambda msg: setattr(self, 'left_wrist_cam_img', msg), 30)
        self.create_subscription(String, '/prompt', lambda msg: setattr(self, 'prompt', msg), 30)
        self.create_subscription(JointState, '/joint_states', lambda msg: setattr(self, 'joint_states', msg), 50)

        # JointTrajectory publishers
        self.arm_controllers={}
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm/command', 25)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper/command', 25)
        self.arm_controllers['arm']=self.arm_publisher
        self.arm_controllers['gripper']=self.gripper_publisher
        
        self.controllers=["arm","gripper"]

        self.get_logger().info("Robot Interface Node initialized with lambda subscribers")
        
    def viewer_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize frame if necessary to match self.frame_size
            if (frame.shape[1], frame.shape[0]) != self.frame_size:
                frame = cv2.resize(frame, self.frame_size)

            # Write the frame to the video file
            self.video_writer.write(frame)
        except Exception as e:
            self.get_logger().error(f"Failed to process frame: {e}")

    # Example method to publish a trajectory
    def act(self,joint_positions):
        for controller in self.controllers: 
            ctrl_joint_positions=list(joint_positions[:len(self.joints[controller])]) if controller=='arm' else \
                list(joint_positions[-1])
            traj = JointTrajectory()
            traj.joint_names = self.joints[controller]
            traj.points.append(
                JointTrajectoryPoint(positions=ctrl_joint_positions, time_from_start=rclpy.duration.Duration(seconds=1).to_msg())
            )
            self.arm_controllers[controller].publish(traj)
            self.get_logger().info("Published arm trajectory")
            
    def joint_state_to_np(self):
        try:
            return np.array(self.joint_states.position, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"[JointState Conversion Failed] {e}")
            return None
    
    def img_msg_to_np(self,msg: Image, encoding: str = 'bgr8') -> np.ndarray:
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        except Exception as e:
            self.get_logger().error(f"[Image Conversion Failed] {e}")
            return None
    
    def get_observation(self) -> dict:
        return {
            "state": self.joint_state_to_np(self.joint_states),
            "images": {
                "observation/exterior_image_1_left":self.img_msg_to_np(self.left_ext_cam_img),
                "observation/wrist_image_left":self.img_msg_to_np(self.left_wrist_cam_img),
                "observation/joint_position": self.joint_state_to_np(self.joint_states)[:-1],
                "observation/gripper_position": self.joint_state_to_np(self.joint_states)[-1],
                },
            "prompt": self.prompt,
        }
    
    def log(self,log_msg):
        self.get_logger().info(log_msg)
    
    def save_video(self):
        if self.video_writer is not None:
            self.video_writer.release()
        
