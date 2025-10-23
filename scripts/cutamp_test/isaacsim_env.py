# Third Party
from isaacsim import SimulationApp

import sys
import numpy as np

# Robot
ROBOT_PRIM_PATH = "/World/robot"

CONFIG = {"renderer": "RaytracedLighting", "headless": False}

# and creation of ROS components
simulation_app = SimulationApp(CONFIG)

import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.utils import extensions, viewports

# enable extension
extensions.enable_extension("isaacsim.ros2.bridge")

from task import Task
from action_graph import create_robot_control_graph
# from isaacsim_gr00t import IsaacSimGr00t

simulation_app.update()

world = World(stage_units_in_meters=1.0)

# # Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

task = Task(name="task")
world.add_task(task)
world.reset() # Setup Scene

simulation_app.update()

create_robot_control_graph(ROBOT_PRIM_PATH)

simulation_app.update()

robot = world.scene.get_object("franka")
robot.post_reset()

# controller = world.scene.get_object("task_space_impedance")

# isaacsim_gr00t = IsaacSimGr00t()
# gr00t_policy = isaacsim_gr00t.setup_gr00t_policy()

world.initialize_physics()
world.play()

frame = 0
while simulation_app.is_running():

    # # Run with a fixed step size
    world.step(render=True)

    if world.is_playing():

        og.Controller.set(og.Controller.attribute("/ActionGraph/RobotControl/OnImpulseEvent.state:enableImpulse"), True)

        observations = task.get_observations()
        joint_pos = observations[robot.name]["joint_position"]
        joint_vel = observations[robot.name]["joint_velocity"]
        ee_pos = observations[robot.name]["end_effector_position"]
        ee_ori = observations[robot.name]["end_effector_orientation"]
        ee_lin_vel = observations[robot.name]["end_effector_linear_velocity"]
        ee_ang_vel = observations[robot.name]["end_effector_angular_velocity"]
        jacobian = observations[robot.name]["jacobian"]
        arm_mass_matrix = observations[robot.name]["arm_mass_matrix"]
        gravity = observations[robot.name]["gravity"]

        # gr00t_observations =isaacsim_gr00t.process_observations(observations)

        # gr00t_actions = gr00t_policy.get_action(gr00t_observations)

        # groot_actions_pos = torch.tensor(gr00t_actions["action.eef_position_delta"], dtype=torch.float32, device=self.device)
        # groot_actions_rot = torch.tensor(gr00t_actions["action.eef_rotation_delta"], dtype=torch.float32, device=self.device)
        # groot_actions_delta_pose = torch.cat(
        #     (groot_actions_pos, groot_actions_rot), dim=-1
        # )

        # action = controller.forward(
        #     joint_pos=joint_pos,
        #     joint_vel=joint_vel,
        #     ee_pos=ee_pos,
        #     ee_ori=ee_ori,
        #     ee_lin_vel=ee_lin_vel,
        #     ee_ang_vel=ee_ang_vel,
        #     jacobian=jacobian,
        #     arm_mass_matrix=arm_mass_matrix,
        #     gravity=gravity
        # )
        # controller.apply_action(action)

        

        



    frame += 1

# world.stop()
simulation_app.close()