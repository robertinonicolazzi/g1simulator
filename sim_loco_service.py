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

    def start_rpc_server(self):
        """Initializes and starts the RPC server."""
        # Initialize SDK [Source 14]
        # Use loopback for local simulation
        ChannelFactoryInitialize(0, "lo") 

        # Create RPC Server [Source 77]
        # 'loco' is the channel name G1 LocoClient expects
        self.server = Server(ChannelFactoryInitialize(0, "lo")) 
        
        # Create Stub and register the API name "LocoApi"
        # This string "LocoApi" MUST match what is defined in unitree_sdk2py/g1/loco/g1_loco_client.py
        self.stub = ServerStub("sport", self.server)
        
        # Register methods
        self.stub.AddMethod("Move", self.Move)
        self.stub.AddMethod("Stop", self.Stop)
        
        # Start Server
        self.server.Start()
        print("[SimLoco] RPC Server is running. LocoClient can now connect.")
