#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
import subprocess
import json
import threading
import os
import signal
import time
import socket
import yaml
from typing import Optional, Dict, List, Any
from dataclasses import dataclass, field
from enum import Enum

class NodeStatus(Enum):
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    FAILED = "failed"

@dataclass
class NodeConfig:
    name: str
    enabled: bool
    launch_command: List[str]
    args: List[str] = field(default_factory=list)
    description: str = ""
    restart_on_failure: bool = True
    health_check: Dict[str, Any] = field(default_factory=dict)
    restart_attempts: int = 0
    max_restart_attempts: int = 3

@dataclass
class ProcessInfo:
    name: str
    process: subprocess.Popen
    start_time: float
    status: NodeStatus = NodeStatus.STARTING
    restart_count: int = 0

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor')
        self.get_logger().info("Supervisor node starting...")
        
        # Parameters
        self.declare_parameter('config_file', '')
        
        # Node enable/disable override parameters (higher priority than config file)
        # Use boolean parameters with None as default (None means use config file)
        self.declare_parameter('fast_livo_enabled', None)
        self.declare_parameter('foxglove_enabled', None)
        self.declare_parameter('livox_enabled', None)
        self.declare_parameter('mvs_enabled', None)
        self.declare_parameter('calibration_enabled', None)
        self.declare_parameter('livox2pc_enabled', None)
        
        # Load configuration
        self.config_file = self.get_parameter('config_file').value
        self.config = self._load_config()
        
        # Initialize data structures
        self.nodes_config: Dict[str, NodeConfig] = {}
        self.processes: Dict[str, ProcessInfo] = {}
        self.record_meta = {
            "running": False,
            "bag_name": None,
            "end_ts": None
        }
        self.lock = threading.Lock()
        self.hostname = socket.gethostname()
        
        # Load node configurations with parameter overrides
        self._load_node_configs_with_overrides()
        
        # Publishers and Subscribers
        self.command_sub = self.create_subscription(
            String, '/supervisor/command', self.on_command, 10
        )
        self.status_pub = self.create_publisher(
            String, '/supervisor/status', 10
        )
        
        # Timers
        self.create_timer(
            self.config['supervisor']['status_publish_interval'],
            self._publish_status
        )
        self.create_timer(
            self.config['supervisor']['process_monitor_interval'],
            self._monitor_processes
        )
        self.create_timer(
            self.config['supervisor']['health_check_interval'],
            self._health_check
        )
        
        self.get_logger().info(f"Supervisor initialized with {len(self.nodes_config)} nodes")
        
        # Start enabled nodes
        self._start_enabled_nodes()
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        config_file = self.config_file
        if not config_file or not os.path.exists(config_file):
            self.get_logger().warn(f"Config file not found: {config_file}, using defaults")
            return self._get_default_config()
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.get_logger().info(f"Loaded config from {config_file}")
                return config
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """Return default configuration."""
        return {
            'nodes': {},
            'recording': {
                'default_topics': [],
                'default_duration': 300,
                'storage_path': '/tmp/rosbags'
            },
            'supervisor': {
                'status_publish_interval': 1.0,
                'process_monitor_interval': 5.0,
                'health_check_interval': 10.0,
                'max_restart_attempts': 3,
                'restart_delay': 2.0
            },
            'security': {
                'require_auth_token': False,
                'allowed_tokens': []
            }
        }
    
    def _load_node_configs_with_overrides(self):
        """Load node configurations from config with parameter overrides."""
        nodes_config = self.config.get('nodes', {})
        
        # Get parameter overrides (now boolean or None)
        fast_livo_enabled_param = self.get_parameter('fast_livo_enabled').value
        foxglove_enabled_param = self.get_parameter('foxglove_enabled').value
        livox_enabled_param = self.get_parameter('livox_enabled').value
        mvs_enabled_param = self.get_parameter('mvs_enabled').value
        calibration_enabled_param = self.get_parameter('calibration_enabled').value
        livox2pc_enabled_param = self.get_parameter('livox2pc_enabled').value
        
        # Helper function to parse parameter value
        def parse_param_value(param_value, default_enabled):
            """Parse parameter value: None means use config, True/False override."""
            if param_value is None:
                return default_enabled  # Use config value
            else:
                # param_value is already a boolean (True/False)
                return param_value
        
        # Process each node configuration
        for name, config in nodes_config.items():
            default_enabled = config.get('enabled', False)
            final_enabled = default_enabled
            
            # Apply parameter override if specified
            if name == 'fast_livo':
                final_enabled = parse_param_value(fast_livo_enabled_param, default_enabled)
            elif name == 'foxglove':
                final_enabled = parse_param_value(foxglove_enabled_param, default_enabled)
            elif name == 'livox':
                final_enabled = parse_param_value(livox_enabled_param, default_enabled)
            elif name == 'mvs':
                final_enabled = parse_param_value(mvs_enabled_param, default_enabled)
            elif name == 'calibration':
                final_enabled = parse_param_value(calibration_enabled_param, default_enabled)
            elif name == 'livox2pc':
                final_enabled = parse_param_value(livox2pc_enabled_param, default_enabled)
            
            # Always create node config, even if disabled (allows manual start via command)
            node_config = NodeConfig(
                name=name,
                enabled=final_enabled,
                launch_command=config['launch_command'],
                args=config.get('args', []),
                description=config.get('description', ''),
                restart_on_failure=config.get('restart_on_failure', True),
                health_check=config.get('health_check', {}),
                max_restart_attempts=self.config['supervisor']['max_restart_attempts']
            )
            self.nodes_config[name] = node_config
            
            if final_enabled:
                self.get_logger().info(f"Node '{name}' enabled (config: {default_enabled}, param override: {final_enabled != default_enabled})")
            else:
                self.get_logger().info(f"Node '{name}' disabled but available for manual start (config: {default_enabled}, param override: {final_enabled != default_enabled})")
    
    def _load_node_configs(self):
        """Load node configurations from config (legacy method, kept for compatibility)."""
        self._load_node_configs_with_overrides()
    
    def _start_enabled_nodes(self):
        """Start all enabled nodes."""
        for name, config in self.nodes_config.items():
            if config.enabled:
                self.get_logger().info(f"Starting enabled node: {name}")
                self._start_node(name)
                time.sleep(1)  # Small delay between starts
    
    def on_command(self, msg: String):
        """Handle incoming commands."""
        try:
            cmd_wrapper = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid JSON: {e} -> {msg.data}")
            return
        
        # Security check
        if not self._check_security(cmd_wrapper):
            return
        
        # Target check
        target = cmd_wrapper.get("target", None)
        if target and target != self.hostname:
            self.get_logger().info(f"Ignoring command for target {target} (we are {self.hostname})")
            return
        
        action = cmd_wrapper.get("command")
        payload = cmd_wrapper.get("payload", {}) or {}
        
        self.get_logger().info(f"Received command: {action}")
        
        try:
            if action == "start_node":
                node_name = payload.get("node")
                if node_name:
                    self._start_node(node_name)
                else:
                    self.get_logger().warn("start_node command missing node name")
            
            elif action == "stop_node":
                node_name = payload.get("node")
                if node_name:
                    self._stop_node(node_name)
                else:
                    self.get_logger().warn("stop_node command missing node name")
            
            elif action == "start_all":
                self._start_all_nodes()
            
            elif action == "stop_all":
                self._stop_all_nodes()
            
            elif action == "start_record":
                topics = payload.get("topics", self.config['recording']['default_topics'])
                # Generate intuitive filename with date and time
                from datetime import datetime
                default_name = datetime.now().strftime("rosbag_%Y%m%d_%H%M%S")
                bag_name = payload.get("name", default_name)
                duration = payload.get("duration", self.config['recording']['default_duration'])
                storage_path = self.config['recording']['storage_path']
                
                os.makedirs(storage_path, exist_ok=True)
                bag_path = os.path.join(storage_path, bag_name)
                
                cmdline = ["ros2", "bag", "record", "-o", bag_path] + topics
                self._start_process("rosbag", cmdline)
                
                with self.lock:
                    self.record_meta["running"] = True
                    self.record_meta["bag_name"] = bag_name
                    self.record_meta["end_ts"] = time.time() + int(duration) if duration else None
                
                if duration:
                    threading.Timer(int(duration), lambda: self._stop_process("rosbag")).start()
            
            elif action == "stop_record":
                self._stop_process("rosbag")
            
            elif action == "restart_node":
                node_name = payload.get("node")
                if node_name:
                    self._stop_node(node_name)
                    time.sleep(1)
                    self._start_node(node_name)
            
            elif action == "get_status":
                # Status is published periodically, but we can trigger immediate publish
                self._publish_status()
            
            else:
                self.get_logger().warn(f"Unknown command: {action}")
        
        except Exception as e:
            self.get_logger().error(f"Error executing command {action}: {e}")
    
    def _check_security(self, cmd_wrapper: Dict) -> bool:
        """Check command security."""
        if not self.config['security']['require_auth_token']:
            return True
        
        incoming_token = cmd_wrapper.get("auth_token")
        allowed_tokens = self.config['security']['allowed_tokens']
        
        if incoming_token in allowed_tokens:
            return True
        
        self.get_logger().warn("Command rejected: invalid auth token")
        return False
    
    def _start_node(self, node_name: str):
        """Start a specific node."""
        config = self.nodes_config.get(node_name)
        if not config:
            self.get_logger().error(f"Node {node_name} not found in configuration")
            return
        
        with self.lock:
            if node_name in self.processes:
                self.get_logger().info(f"Node {node_name} is already running")
                return
        
        # Build command
        cmd = config.launch_command.copy()
        if config.args:
            cmd.extend(config.args)
        
        self._start_process(node_name, cmd)
    
    def _stop_node(self, node_name: str):
        """Stop a specific node."""
        self._stop_process(node_name)
    
    def _start_all_nodes(self):
        """Start all configured nodes."""
        for node_name in self.nodes_config:
            self._start_node(node_name)
            time.sleep(0.5)  # Small delay between starts
    
    def _stop_all_nodes(self):
        """Stop all running nodes."""
        with self.lock:
            nodes_to_stop = list(self.processes.keys())
        
        for node_name in nodes_to_stop:
            if node_name != "rosbag":  # Don't stop rosbag with this command
                self._stop_node(node_name)
                time.sleep(0.5)
    
    def _start_process(self, name: str, cmd: List[str]):
        """Start a process."""
        with self.lock:
            if name in self.processes:
                return
        
        self.get_logger().info(f"Starting {name}: {' '.join(cmd)}")
        
        try:
            # Create log directory
            log_dir = os.path.join(os.path.expanduser("~"), ".ros", "supervisor_logs")
            os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, f"{name}_{int(time.time())}.log")
            
            with open(log_file, 'w') as f:
                f.write(f"Command: {' '.join(cmd)}\n")
                f.write(f"Start time: {time.ctime()}\n")
                f.write("-" * 80 + "\n")
            
            # Start process
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                text=True
            )
            
            process_info = ProcessInfo(
                name=name,
                process=proc,
                start_time=time.time(),
                status=NodeStatus.RUNNING
            )
            
            with self.lock:
                self.processes[name] = process_info
            
            # Start output forwarding threads
            threading.Thread(
                target=self._forward_output,
                args=(name, proc, log_file),
                daemon=True
            ).start()
            
            self.get_logger().info(f"{name} started with PID {proc.pid}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start {name}: {e}")
    
    def _stop_process(self, name: str):
        """Stop a process."""
        with self.lock:
            process_info = self.processes.get(name)
            if not process_info:
                if name == "rosbag":
                    self.record_meta["running"] = False
                    self.record_meta["end_ts"] = None
                self.get_logger().info(f"{name} not running")
                return
        
        self.get_logger().info(f"Stopping {name} (PID: {process_info.process.pid})")
        process_info.status = NodeStatus.STOPPING
        
        try:
            # Send SIGTERM to process group
            os.killpg(os.getpgid(process_info.process.pid), signal.SIGTERM)
        except Exception as e:
            self.get_logger().warn(f"Error sending SIGTERM to {name}: {e}")
            try:
                process_info.process.terminate()
            except Exception as e2:
                self.get_logger().error(f"Error terminating {name}: {e2}")
        
        # Wait for process to terminate
        def wait_and_clean():
            try:
                process_info.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process_info.process.pid), signal.SIGKILL)
                except Exception:
                    pass
            
            with self.lock:
                if name in self.processes:
                    del self.processes[name]
            
            if name == "rosbag":
                with self.lock:
                    self.record_meta["running"] = False
                    self.record_meta["end_ts"] = None
            
            self.get_logger().info(f"{name} stopped")
        
        threading.Thread(target=wait_and_clean, daemon=True).start()
    
    def _forward_output(self, name: str, proc: subprocess.Popen, log_file: str):
        """Forward process output to logs and file."""
        try:
            with open(log_file, 'a') as log_f:
                # Read stdout
                for line in iter(proc.stdout.readline, ''):
                    if line:
                        line = line.rstrip()
                        self.get_logger().info(f"[{name}] {line}")
                        log_f.write(f"{line}\n")
                        log_f.flush()
                
                # Read stderr
                for line in iter(proc.stderr.readline, ''):
                    if line:
                        line = line.rstrip()
                        self.get_logger().error(f"[{name}] ERR: {line}")
                        log_f.write(f"ERR: {line}\n")
                        log_f.flush()
        
        except Exception as e:
            self.get_logger().warn(f"Output forwarding for {name} stopped: {e}")
    
    def _monitor_processes(self):
        """Monitor running processes."""
        processes_to_restart = []
        
        with self.lock:
            to_remove = []
            for name, process_info in list(self.processes.items()):
                ret = process_info.process.poll()
                if ret is not None:
                    self.get_logger().warn(f"Process {name} exited with code {ret}")
                    
                    # Always remove the process from the dictionary
                    to_remove.append(name)
                    
                    # Check if we should restart
                    config = self.nodes_config.get(name)
                    if config and config.restart_on_failure:
                        config.restart_attempts += 1
                        if config.restart_attempts <= config.max_restart_attempts:
                            self.get_logger().info(f"Restarting {name} (attempt {config.restart_attempts}/{config.max_restart_attempts})")
                            # Schedule restart outside the lock
                            processes_to_restart.append((name, config.restart_attempts))
                        else:
                            self.get_logger().error(f"Max restart attempts reached for {name}")
            
            # Remove all exited processes
            for name in to_remove:
                if name in self.processes:
                    del self.processes[name]
            
            # Check recording end time
            if self.record_meta["running"] and self.record_meta["end_ts"]:
                if time.time() >= self.record_meta["end_ts"]:
                    self.get_logger().info("Recording duration reached, stopping rosbag")
                    self._stop_process("rosbag")
        
        # Restart processes outside the lock to avoid blocking
        for name, attempt in processes_to_restart:
            # Wait for restart delay
            time.sleep(self.config['supervisor']['restart_delay'])
            self._start_node(name)
    
    def _health_check(self):
        """Perform health checks on running nodes."""
        # This is a placeholder for health check implementation
        # Could check topic availability, service responses, etc.
        pass
    
    def _publish_status(self):
        """Publish current status."""
        with self.lock:
            # Get node statuses
            node_statuses = {}
            for name, process_info in self.processes.items():
                node_statuses[name] = {
                    "running": True,
                    "pid": process_info.process.pid,
                    "uptime": time.time() - process_info.start_time,
                    "status": process_info.status.value
                }
            
            # Add configured but not running nodes
            for name, config in self.nodes_config.items():
                if name not in node_statuses:
                    node_statuses[name] = {
                        "running": False,
                        "pid": None,
                        "uptime": 0,
                        "status": NodeStatus.STOPPED.value
                    }
            
            # Build status message
            status = {
                "hostname": self.hostname,
                "timestamp": time.time(),
                "nodes": node_statuses,
                "recording": {
                    "running": self.record_meta["running"],
                    "bag_name": self.record_meta["bag_name"],
                    "remaining_seconds": max(0, self.record_meta["end_ts"] - time.time()) if self.record_meta["end_ts"] else None
                },
                "supervisor": {
                    "config_file": self.config_file,
                    "nodes_configured": len(self.nodes_config),
                    "nodes_running": len(self.processes)
                }
            }
        
        msg = String()
        msg.data = json.dumps(status, default=str)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down supervisor...")
    finally:
        # Stop all processes
        with node.lock:
            for name, process_info in list(node.processes.items()):
                try:
                    os.killpg(os.getpgid(process_info.process.pid), signal.SIGTERM)
                except Exception:
                    pass
        
        node.destroy_node()
        rclpy.shutdown()
