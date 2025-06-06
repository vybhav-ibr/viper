import rclpy
from viperX300_description.aloha_sim import RosAlohaSim
from viperX300_description.droid_sim import RosDroidSim
from viperX300_description.agibot_sim import RosAgibotSim
import enum
import tyro
from openpi_client import websocket_client_policy as _websocket_client_policy


class Args:
    host: str = "0.0.0.0"
    port: int = 8000
    env: str = "ROS_ALOHA_SIM"
    num_steps: int = 1000
    prompt: str=""
        
class RosAlohaSimEnv(RosAlohaSim):
    def __init__(self,exp_name):
        super().__init__('ros_aloha_sim')
        self.get_logger().info("ROS_ALOHA_SIM env chosen")

class RosDroidSimEnv(RosDroidSim):
    def __init__(self,exp_name):
        super().__init__('ros_droid_sim')
        self.get_logger().info("ROS_DROID_SIM env chosen")

class RosAgibotSimEnv(RosAgibotSim):
    def __init__(self,exp_name):
        super().__init__('ros_ur_sim')
        self.get_logger().info("ROS_AGIBOT_SIM env chosen")
        
def env_factory(type: str,exp_name: str):
    if type == "ROS_ALOHA_SIM":
        node=RosAlohaSimEnv(exp_name)
    elif type == "ROS_DROID_SIM":
        node=RosDroidSimEnv(exp_name)
    elif type == "ROS_UR_SIM":
        node=RosAgibotSimEnv(exp_name)
    return node        

def main():
    args=tyro.cli(Args)
    rclpy.init()
    node=env_factory(args.env)
    
    policy =_websocket_client_policy.WebsocketClientPolicy(
        host=args.host, 
        port=args.port)

    node.log(f"Server metadata: {policy.get_server_metadata()}")
    for i in range(3):
        obs = node.get_observation()
        policy.infer(obs)
        
    # optional loop:
    current_action_idx=0
    try:
        if current_action_idx ==10:
            current_action_idx=0
            action_chunk=policy.infer(node.get_observation())["actions"]
        node.act(action_chunk[current_action_idx])
        current_action_idx+=1
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node intterupted")
    finally:
        if node is not None:
            node.save_vid()
            node.destroy_node()
    rclpy.shutdown()
    
