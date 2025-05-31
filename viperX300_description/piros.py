# rospi.py

import tyro
# rospi_config.py

import enum
import dataclasses
import logging
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from openpi_client import websocket_client_policy as _websocket_client_policy

__all__ = ["EnvMode", "Args", "RospiNode", "main"]

# ----- Environment Modes and Config -----

class EnvMode(enum.Enum):
    ROS_ALOHA = "aloha"
    ROS_ALOHA_SIM = "aloha_sim"
    ROS_DROID = "droid"
    ROS_DROID_SIM = "droid_sim"

ENV_CONFIG = {
    EnvMode.ROS_ALOHA: {
        'cameras': ['cam_high', 'cam_low', 'cam_left', 'cam_right'],
        'joints': {
            "left": ['left_waist', 'left_shoulder', 'left_elbow', 'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate', 'left_left_finger', 'left_right_finger'],
            "right": ['right_waist', 'right_shoulder', 'right_elbow', 'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate', 'right_left_finger', 'right_right_finger'],
        },
        'controllers': {
            'left':'left_joint_trajectory_controller/joint_trajectory',
            'right':'right_joint_trajectory_controller/joint_trajectory'
        },
    },
    EnvMode.ROS_ALOHA_SIM: lambda: ENV_CONFIG[EnvMode.ROS_ALOHA],
    EnvMode.ROS_DROID: {
        'cameras': ['exterior_left', 'wrist_left'],
        'joints': [f'joint{i}' for i in range(1, 9)],
        'controllers': {'one':'joint_trajectory_controller/joint_trajectory'},
    },
    EnvMode.ROS_DROID_SIM: lambda: ENV_CONFIG[EnvMode.ROS_DROID],
}

# Resolve any lambdas
for k, v in list(ENV_CONFIG.items()):
    if callable(v):
        ENV_CONFIG[k] = v()

# ----- Argument Dataclass -----

@dataclasses.dataclass
class Args:
    host: str = "0.0.0.0"
    port: int = 8000
    env: EnvMode = EnvMode.ROS_ALOHA_SIM
    num_steps: int = 1000
    prompt: str="pick the apples from the trees"

# ----- Shared Converters -----

bridge = CvBridge()

def img_msg_to_np(msg: Image, encoding: str = 'bgr8') -> np.ndarray:
    try:
        return bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
    except Exception as e:
        logging.error(f"[Image Conversion Failed] {e}")
        return None

def joint_state_to_np(msg: JointState) -> np.ndarray:
    try:
        mappings=[5,4,0,1,6,7,2,13,12,8,9,14,15,10]
        orderred_positins=[msg.position[i] for i in mappings]
        return np.array(orderred_positins, dtype=np.float32)
    except Exception as e:
        logging.error(f"[JointState Conversion Failed] {e}")
        return None
    
def np_to_joint_trajectory(nparray,joints: list,controllers: list) ->dict:
    return_dict={}
    
    traj = JointTrajectory()
    for ctrl in controllers:
        traj.joint_names = joints[ctrl]
        for i, _ in enumerate(joints[ctrl]):
            pt = JointTrajectoryPoint()
            pt.positions = list(nparray[:len(joints[ctrl])]+nparray[len(joints[ctrl])+1])
            pt.time_from_start.sec = i + 1
            traj.points.append(pt)
        
        return_dict[ctrl]=traj
    
    return return_dict
    

# ----- ROS Node Class -----

class RospiNode(Node):
    def __init__(self, args: Args):
        super().__init__('rospi_node')
        self.args = args
        self.env = args.env
        self.config = ENV_CONFIG[self.env]
        self.prompt=args.prompt

        # Placeholders
        self.images = {cam: None for cam in self.config["cameras"]}
        self.joint_states = None

        # Subscriptions
        for cam in self.config["cameras"]:
            topic = f"{cam}_image"
            self.create_subscription(
                Image, topic,
                callback=lambda msg, cam=cam: self._on_image(msg, cam),
                qos_profile=15,
            )

        self.create_subscription(
            JointState, 'joint_states',
            callback=self._on_joint_state,
            qos_profile=50,
        )

        # Publishers & timers
        self.publishers = {}
        for ctrl in self.config['controllers']:
            self._setup_trajectory_publisher(ctrl)

    def _on_image(self, msg: Image, cam: str):
        self.images[cam] = msg

    def _on_joint_state(self, msg: JointState):
        self.joint_states = msg

    def _setup_trajectory_publisher(self, topic_name: str):
        pub = self.create_publisher(JointTrajectory, topic_name, 10)
        self.publishers[topic_name] = pub

        # select joint names
        # if "left" in topic_name:
        #     names = [j for j in self.config['joints'] if j.startswith('left_')]
        # elif "right" in topic_name:
        #     names = [j for j in self.config['joints'] if j.startswith('right_')]
        # else:
        #     names = self.config['joints']

        # def timer_callback():
        #     traj = JointTrajectory()
        #     traj.joint_names = names
        #     for i, _ in enumerate(names):
        #         pt = JointTrajectoryPoint()
        #         pt.positions = [0.1 * (i+1)] * len(names)
        #         pt.time_from_start.sec = i + 1
        #         traj.points.append(pt)
        #     pub.publish(traj)
        #     self.get_logger().info(f"Published trajectory to {topic_name}")

        # self.create_timer(0.02, timer_callback)

    def get_observation(self) -> dict:
        if self.env in (EnvMode.ROS_ALOHA, EnvMode.ROS_ALOHA_SIM):
            return {
                "state": joint_state_to_np(self.joint_states),
                "images": {cam: img_msg_to_np(im) for cam, im in self.images.items()},
                "prompt": self.prompt,
            }
        elif self.env in (EnvMode.ROS_DROID,EnvMode.ROS_DROID_SIM):
            # DROID variants
            pos = joint_state_to_np(self.joint_states)
            return {
                "observation/exterior_image_1_left": img_msg_to_np(self.images['exterior_left']),
                "observation/wrist_image_left": img_msg_to_np(self.images['wrist_left']),
                "observation/joint_position": pos[:-1],
                "observation/gripper_position": pos[-1],
                "prompt": self.prompt,
            }
    
    def act(self, action):
        msg_dict=np_to_joint_trajectory(action,self.config['joints'],self.config['controllers'])
        for ctrl in self.config['controllers']:
            self.publishers[ctrl].publish(msg_dict[ctrl])
                
        

# ----- Main Entry Point -----

def main(args: Args):
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = RospiNode(args)
    
    policy =_websocket_client_policy.WebsocketClientPolicy(
        host=args.host, 
        port=args.port)

    logging.info(f"Server metadata: {policy.get_server_metadata()}")
    obs = node.get_observation()
    policy.infer(obs)

    # optional loop:
    current_action_idx=0
    for _ in range(args.num_steps):
        if current_action_idx ==10:
            current_action_idx=0
            action_chunk=policy.infer(node.get_observation())["actions"]
        node.act(action_chunk[current_action_idx])
        current_action_idx+=1
        

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # Parses CLI flags into Args, then hands off to the common main()
    main(tyro.cli(Args))
