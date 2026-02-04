import time
import sys
import json

# Import SDK components based on [13], [31], [32]
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.rpc.server import Server
from unitree_sdk2py.rpc.server_stub import ServerStub

class SimLocoService:
    """
    This service accepts RPC calls from LocoClient and forwards them
    to the simulation via DDS, mimicking the logic in dds_ros2_bridge.
    """
    def __init__(self):
        # 1. Initialize DDS Publisher [Source 13, 17]
        # The simulation listens to this specific topic for velocity commands
        self.cmd_pub = ChannelPublisher("rt/run_command/cmd", String_)
        self.cmd_pub.Init()
        
        # Default height as seen in dds_ros2_bridge [6]
        self.default_height = 0.8
        print("[SimLoco] DDS Publisher initialized on 'rt/run_command/cmd'")
        
        self.server = None
        self.stub = None

    def Move(self, x, y, omega):
        """
        Receives velocity from LocoClient (RPC) and sends to Sim (DDS).
        Args:
            x: Linear velocity X (m/s)
            y: Linear velocity Y (m/s)
            omega: Angular velocity Yaw (rad/s)
        """
        # 2. Format the command string [Source 17]
        # The bridge expects a string representation of a list: [vx, vy, w, height]
        # Note: dds_ros2_bridge flips signs for ROS. 
        # If LocoClient is using standard Robot Frame (X-forward), pass directly.
        cmd_list = [float(x), float(y), float(omega), self.default_height]
        cmd_str = str(cmd_list)

        # 3. Publish to DDS
        msg = String_(data=cmd_str)
        self.cmd_pub.Write(msg)
        
        print(f"[SimLoco] Forwarding: vx={x:.2f}, vy={y:.2f}, w={omega:.2f}")
        return 0  # Return 0 (OK) to the Client

    def Stop(self):
        """Stops the robot."""
        self.Move(0.0, 0.0, 0.0)
        return 0

    def _handle_set_velocity(self, parameter: str):
        """
        Handler for ROBOT_API_ID_LOCO_SET_VELOCITY (7105)
        Parameter is a JSON string: {"velocity": [vx, vy, omega], "duration": ...}
        """
        try:
            data = json.loads(parameter)
            velocity = data.get("velocity", [0.0, 0.0, 0.0])
            vx = float(velocity[0])
            vy = float(velocity[1])
            omega = float(velocity[2])
            
            self.Move(vx, vy, omega)
            return 0, "" # Return code 0 (OK) and empty data string
        except Exception as e:
            print(f"[SimLoco] Error handling SetVelocity: {e}")
            return -1, ""

    def start_rpc_server(self):
        """Initializes and starts the RPC server."""
        # Initialize SDK [Source 14]
        print("[SimLoco] Initialize SDK")
        ChannelFactoryInitialize(0, "lo")
        print("[SimLoco] Initialize SDK done") 

        # Create RPC Server
        # Use simple Server class which allows registering handlers by ID
        print("[SimLoco] Create RPC Server")
        self.server = Server("sport") 
        print("[SimLoco] Create RPC Server done")
        
        # Register Handler for SetVelocity (ID 7105)
        # 7105 is ROBOT_API_ID_LOCO_SET_VELOCITY as per SDK
        ROBOT_API_ID_LOCO_SET_VELOCITY = 7105
        self.server._RegistHandler(ROBOT_API_ID_LOCO_SET_VELOCITY, self._handle_set_velocity, False)
        print(f"[SimLoco] Registered handler for ID {ROBOT_API_ID_LOCO_SET_VELOCITY}")
        
        # Start Server
        print("[SimLoco] Start Server")
        self.server.Start()
        print("[SimLoco] RPC Server is running. LocoClient can now connect.")

if __name__ == "__main__":
    service = SimLocoService()
    service.start_rpc_server()
    while True:
        time.sleep(1)

