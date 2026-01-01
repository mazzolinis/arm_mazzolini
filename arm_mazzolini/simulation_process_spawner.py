#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from typing import Any
from rclpy.timer import Timer

import subprocess
import signal
import os
from dataclasses import dataclass
from typing import List, Optional

# Definition of class node that has to be spawned
@dataclass
class SpawnTarget:
    name: str
    package: str
    executable: str
    extra_args: List[str]
    delay: float        # seconds of delay after simulation started
    params_file: Optional[str] = None
    inline_params: Optional[dict[str, Any]] = None
    spawned: bool = False     # check if already spawned
    timer: Optional[Timer] = None

# =====================================================
# This node spawns controllers and other nodes
# USE IT ONLY IN GAZEBO SIMULATION
# =====================================================
class SimulationProcessSpawner(Node):

    def __init__(self):
        super().__init__('simulation_process_spawner')

        self.get_params()
        self.clock = Clock(clock_type=ClockType.ROS_TIME)
        self.last_time = self.clock.now()
        self.started = False
        self.processes = []

        # Config
        self.spawn_targets = self.build_spawn_targets()

        # Checking if simulation started
        self.play_timer = self.create_timer(0.1, self.check_clock)

        # Checking if all nodes are still running
        self.monitor_timer = self.create_timer(1.0, self.monitor_processes)

        self.get_logger().info('Waiting for simulation to start ...')

    # -------------------------------------------------

    def check_clock(self):
        now = self.clock.now()

        if not self.started and now > self.last_time:
            self.started = True
            self.play_timer.cancel()
            self.get_logger().info('Simulation started. Scheduling nodes.')
            self.schedule_all()

        self.last_time = now

    # -------------------------------------------------

    def schedule_all(self):
        for target in self.spawn_targets:
            timer = self.create_timer(
                target.delay,
                lambda t=target: self.spawn_once(t)
            )
            target.timer = timer
    # -------------------------------------------------

    def spawn_once(self, target: SpawnTarget):
        if target.spawned:
            return

        self.spawn(target)
        target.spawned = True

        # Clean up timer to avoid useless callbacks
        if target.timer is not None:
            target.timer.cancel()
            target.timer = None

    # -------------------------------------------------

    def spawn(self, target: SpawnTarget):
        cmd = ['ros2', 'run', target.package, target.executable]
        cmd.extend(target.extra_args)
        cmd.extend(['--ros-args', '-p', 'use_sim_time:=true']) # This is the reason for simulation only

        if target.inline_params:
            for key, value in target.inline_params.items():
                cmd.extend(['-p', f'{key}:={value}'])

        if target.params_file:
            cmd.extend(['--params-file', target.params_file])

        self.get_logger().info(f'Spawning {target.name}')

        proc = subprocess.Popen(
            cmd,
            stdout=None,  # change these to subprocess.PIPE to intercept output
            stderr=None,
            preexec_fn=os.setsid
        )

        self.processes.append(proc)

    # -------------------------------------------------

    def monitor_processes(self):
        for proc in self.processes:
            if proc.poll() is not None:
                self.get_logger().error(
                    f'Process with PID {proc.pid} has terminated with code {proc.returncode}.'
                )
                self.processes.remove(proc)

    # -------------------------------------------------

    def destroy_node(self):
        # Shut spawned processes, necessary to avoid orphan processes
        for proc in self.processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        super().destroy_node()

    # ------------------- PARAMETERS ARE WRITTEN HERE -----------------------
   
    def get_params(self):

        self.declare_parameter('controller_yaml', ' ')
        self.controller_yaml = self.get_parameter('controller_yaml').get_parameter_value().string_value
        self.get_logger().info(f'Using controller YAML: {self.controller_yaml}')
        self.declare_parameter('world_height', 0.0)
        self.world_height = self.get_parameter('world_height').get_parameter_value().double_value

    # ------------------- NODES ARE WRITTEN HERE -----------------------

    def build_spawn_targets(self):

        controller_manager = '/controller_manager'

        targets = []

            # controllers
        targets.append(
            SpawnTarget(
                name='joint_state_broadcaster',
                package='controller_manager',
                executable='spawner',
                extra_args=['joint_state_broadcaster'],
                delay=0.1   # Avoid using 0.0
            )
        )
        targets.append(
            SpawnTarget(
                name='joint_trajectory_controller',
                package='controller_manager',
                executable='spawner',
                extra_args=['joint_trajectory_controller',
                            '--controller-manager', controller_manager],
                params_file=self.controller_yaml,
                delay=0.1
            )
        )
        targets.append(
            SpawnTarget(
                name='diff_drive_controller',
                package='controller_manager',
                executable='spawner',
                extra_args=['diff_drive_controller',
                            '--controller-manager', controller_manager],
                params_file=self.controller_yaml,
                delay=0.1
            )
        )

        #  kinematic node
        targets.append(
            SpawnTarget(
                name='kinematic_node',
                package='arm_mazzolini',
                executable='kinematic_node',
                extra_args=[],
                params_file=self.controller_yaml,
                delay=1.0
            )
        )

        # target spawner
        targets.append(
            SpawnTarget(
                name='target_spawner',
                package='arm_mazzolini',
                executable='target_spawner',
                extra_args=[],
                inline_params={'world_height': self.world_height},
                delay=2.0
            )
        )
        
        return targets

# =====================================================

def main(args=None):
    rclpy.init(args=args)
    node = SimulationProcessSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
