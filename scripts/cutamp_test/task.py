from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import omni
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleXFormPrim, SingleRigidPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units, add_reference_to_stage
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.camera import Camera, SingleViewDepthSensor
from isaacsim.robot.manipulators.grippers import Gripper, ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from pxr import Gf
import isaacsim.core.utils.numpy.rotations as rot_utils
# import task_space_impedance

class Task(ABC, BaseTask):
    
    def __init__(
        self,
        name: str,
    ) -> None:
        BaseTask.__init__(self, name=name)
        self._robot = None
        self._controller = None
        self._asset_root_path = get_assets_root_path()
        if self._asset_root_path is None:
            raise Exception("Could not find Isaac Sim assets folder")
        self._robot_prim_path = "/World/robot"
        self._robot_asset_path = self._asset_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        self._robot_end_effector_link_name = "panda_rightfinger"
        self._is_fixed_base = True  # Set to False if the robot is not fixed to the base
        return

    def set_up_scene(self, scene: Scene) -> None:
        super().set_up_scene(scene)

        scene.add_default_ground_plane(z_position = -0.64)

        table = FixedCuboid(
            prim_path="/World/table",
            name="table",
            scale = np.array([1.4, 0.91, 0.64]),
            translation=np.array([0.45, 0.0, -0.32]),
        )

        # Define the robot
        self._robot = self.set_robot()

        # cfg
        # self._controller = self.set_controller()

        # # Add the robots and objects to the scene
        scene.add(table)
        scene.add(self._robot)
        # scene.add(self._controller)
    

    def set_robot(self) -> SingleManipulator:

        robot_prim_path = find_unique_string_name(
            initial_name=self._robot_prim_path, is_unique_fn=lambda x: not is_prim_path_valid(x)
        )

        robot = add_reference_to_stage(usd_path=self._robot_asset_path, prim_path=robot_prim_path)
        robot.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
        robot.GetVariantSet("Mesh").SetVariantSelection("Quality")

        gripper = ParallelGripper(
            end_effector_prim_path=robot_prim_path + "/" + self._robot_end_effector_link_name,
            joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
            joint_opened_positions=np.array([0.05, 0.05]),
            joint_closed_positions=np.array([0.0, 0.0]),
            action_deltas=np.array([0.01, 0.01]),
        )
        
        manipulator = SingleManipulator(
            prim_path=robot_prim_path,
            name="franka",
            end_effector_prim_path=robot_prim_path + "/" + self._robot_end_effector_link_name,
            gripper=gripper,
        )

        manipulator.set_joints_default_state(
            positions = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785, # Arm
                                  0.0, 0.0]), # Gripper joint position
        )

        return manipulator
    
    def set_controller(self, controller_cfg):
        pass





    def set_object(
        self, usd_path: str, prim_path: Optional[str] = None, name: Optional[str] = None,
        scale: Optional[tuple] = None, position: Optional[tuple] = None, orientation: Optional[list] = None, degrees=True,
    ) -> None:
        
        # Create Peg, Hole 

        pass
        
        # prim_path = prim_path or find_unique_string_name(
        #     initial_name="/World/objects/" + name if name else "/World/objects/object",
        #     is_unique_fn=lambda x: not is_prim_path_valid(x)
        # )
        # name = name or prim_path.split("/")[-1]
        # if scale is None:
        #     scale = (1.0, 1.0, 1.0)
        # if position is None:
        #     position = (0.0, 0.0, 0.0)
        # if orientation is None:
        #     orientation = [1.0, 0.0, 0.0, 0.0]
        
        # add_reference_to_stage(
        #     usd_path=usd_path,
        #     prim_path=prim_path,
        # )
        # xform_prim = SingleXFormPrim(
        #     prim_path=prim_path,
        #     name=name,
        #     scale=scale
        # )
        # xform_prim.set_world_pose(
        #     position=position,
        #     orientation=rot_utils.euler_angles_to_quats(np.array(orientation), degrees=degrees),
        # )

    def get_observations(self) -> dict:
        joints_state = self._robot.get_joints_state()
        end_effector_position, end_effector_orientation = self._robot.end_effector.get_local_pose()
        end_effector_linear_velocity = self._robot.end_effector.get_linear_velocity()
        end_effector_angular_velocity = self._robot.end_effector.get_angular_velocity()

        if self._is_fixed_base:
            ee_index = self._robot._articulation_view.get_link_index(self._robot_end_effector_link_name) - 1
        else:
            ee_index = self._robot._articulation_view.get_link_index(self._robot_end_effector_link_name)

        jacobian = self._robot._articulation_view.get_jacobians(indices=[0])
        jacobian = jacobian.squeeze()[ee_index, :, :7] # (6, 7)

        arm_mass_matrix = self._robot._articulation_view.get_mass_matrices(indices=[0])
        arm_mass_matrix = arm_mass_matrix.squeeze()[:7, :7] # (7, 7)

        gravity = self._robot._articulation_view.get_generalized_gravity_forces(
            indices=[0], joint_indices=np.arange(0, 7)
        ).squeeze() # (7,)
        
        
        # joint_forces = self._robot.get_measured_joint_forces(-1).flatten()
        return {
            self._robot.name: {
                "joint_position": joints_state.positions,
                "joint_velocity": joints_state.velocities,
                "end_effector_position": end_effector_position,
                "end_effector_orientation": end_effector_orientation,
                "end_effector_linear_velocity": end_effector_linear_velocity,
                "end_effector_angular_velocity": end_effector_angular_velocity,
                "jacobian": jacobian,
                "arm_mass_matrix": arm_mass_matrix,
                "gravity": gravity,
            },
        }
        #     "sensor": {
        #         "FT": {
        #             "force": joint_forces[0:3],
        #             "torque": joint_forces[3:6],
        #         },
        #     },

        # def compute(
        #     self,
        #     joint_pos: torch.Tensor,
        #     joint_vel: torch.Tensor,
        #     current_ee_pose_b: torch.Tensor,
        #     current_ee_vel_b: torch.Tensor,
        #     jacobian_b: torch.Tensor,
        #     arm_mass_matrix: torch.Tensor,
        #     gravity: torch.Tensor,
        # ) -> tuple[torch.Tensor, torch.Tensor]:
        
    
    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        return
    
    def post_reset(self) -> None:
        # from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

        # if isinstance(self._robot.gripper, ParallelGripper):
        #     self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)
        return


