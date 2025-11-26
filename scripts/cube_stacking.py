#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#

import isaacsim
import numpy as np

np.set_printoptions(suppress=True)

import argparse

parser = argparse.ArgumentParser()

parser.add_argument(
    "--headless_mode",
    type=str,
    default=None,
    help="To run headless, use one of [native, websocket], webrtc might not work.",
)

parser.add_argument(
    "--constrain_grasp_approach",
    action="store_true",
    help="When True, approaches grasp with fixed orientation and motion only along z axis.",
    default=False,
)
args = parser.parse_args()

# Third Party
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless_mode is not None,
    "width": "1920",
    "height": "1080",
})

# Standard Library
from typing import Optional

# Third Party
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

from sdl.utils.helper import add_extensions
from sdl.controllers import CuroboCubeStackController
from sdl.tasks.cube_stack import CuroboCubeStackTask


robot_prim_path = "/World/FR5/base_link"
ignore_substring = ["FR5", "TargetCube", "material", "Plane"]
my_world = World(stage_units_in_meters=1.0)
stage = my_world.stage
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

my_task = CuroboCubeStackTask()
my_world.add_task(my_task)
my_world.reset()
robot_name = my_task.get_params()["robot_name"]["value"]
my_fr5 = my_world.scene.get_object(robot_name)
my_controller = CuroboCubeStackController(
    my_world=my_world, my_task=my_task, constrain_grasp_approach=args.constrain_grasp_approach
)
articulation_controller = my_fr5.get_articulation_controller()
set_camera_view(eye=[2, 0, 1], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")
wait_steps = 8

my_fr5.set_solver_velocity_iteration_count(4)
my_fr5.set_solver_position_iteration_count(124)
my_world._physics_context.set_solver_type("TGS")
initial_steps = 100
################################################################
print("Start simulation...")
robot = my_fr5
print(
    my_world._physics_context.get_solver_type(),
    robot.get_solver_position_iteration_count(),
    robot.get_solver_velocity_iteration_count(),
)
print(my_world._physics_context.use_gpu_pipeline)
print(articulation_controller.get_gains())
print(articulation_controller.get_max_efforts())
robot = my_fr5
print("**********************")
if False:
    robot.enable_gravity()
    articulation_controller.set_gains(
        kps=np.array(
            [100000000, 6000000.0, 10000000, 600000.0, 25000.0, 15000.0, 50000.0, 6000.0, 6000.0]
        )
    )

    articulation_controller.set_max_efforts(
        values=np.array([100000, 52.199997, 100000, 52.199997, 7.2, 7.2, 7.2, 50.0, 50])
    )

print("Updated gains:")
print(articulation_controller.get_gains())
print(articulation_controller.get_max_efforts())
# exit()
my_fr5.gripper.open()
for _ in range(wait_steps):
    my_world.step(render=True)
my_task.reset()
task_finished = False
observations = my_world.get_observations()
my_task.get_pick_position(observations)

i = 0

add_extensions(simulation_app, args.headless_mode)

while simulation_app.is_running():
    my_world.step(render=True)  # necessary to visualize changes
    i += 1

    if my_world.is_playing():

        if task_finished or i < initial_steps:
            continue

        if not my_controller.init_curobo:
            my_controller.reset(ignore_substring, robot_prim_path)

        step_index = my_world.current_time_step_index
        observations = my_world.get_observations()
        sim_js = my_fr5.get_joints_state()

        if my_controller.reached_target(observations):
            if my_fr5.gripper.get_joint_positions()[0] > 0.3:  # reached placing target
                my_fr5.gripper.open()
                for _ in range(wait_steps):
                    my_world.step(render=True)
                my_controller.detach_obj()
                my_controller.update(
                    ignore_substring, robot_prim_path
                )  # update world collision configuration
                task_finished = my_task.update_task()
                if task_finished:
                    print("\nTASK DONE\n")
                    for _ in range(wait_steps):
                        my_world.step(render=True)
                    continue
                else:
                    my_task.get_pick_position(observations)

            else:  # reached picking target
                my_fr5.gripper.close()
                for _ in range(wait_steps):
                    my_world.step(render=True)
                sim_js = my_fr5.get_joints_state()
                my_controller.update(ignore_substring, robot_prim_path)
                my_controller.attach_obj(sim_js, my_fr5.dof_names)
                my_task.get_place_position(observations)

        else:  # target position has been set
            sim_js = my_fr5.get_joints_state()
            art_action = my_controller.forward(sim_js, my_fr5.dof_names)
            if art_action is not None:
                articulation_controller.apply_action(art_action)
                # for _ in range(2):
                #    my_world.step(render=False)

simulation_app.close()
