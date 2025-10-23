import numpy as np
from abc import ABC, abstractmethod
from typing import Optional

from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.numpy.maths import matmul
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats


class BasePickPlace(ABC, BaseTask):
    def __init__(
        self,
        name: str,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseTask.__init__(self, name=name, offset=offset)
        self.robot = None
        self.object = None
        self.object_initial_position = None
        self.object_initial_orientation = None
        self.place_position = None
        self.place_orientation = None

        self.target_position = None
        self.target_orientation = None
        self.grasp_offset = None
        self.object_size = None
        self.object_in_hand = None

        if self.object_size is None:
            self.object_size = np.array([0.045, 0.045, 0.045]) / get_stage_units()
        if self.object_initial_position is None:
            self.object_initial_position = np.array([0.3, 0.3, 0]) / get_stage_units()
            self.object_initial_position[2] = self.object_size[2] / 2
        if self.object_initial_orientation is None:
            self.object_initial_orientation = np.array([1, 0, 0, 0])
        return
    
    def set_up_scene(self, scene: Scene) -> None:
        """[summary]

        Args:
            scene (Scene): [description]
        """
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        object_prim_path = find_unique_string_name(
            initial_name="/World/Object", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        object_name = find_unique_string_name(initial_name="object", is_unique_fn=lambda x: not self.scene.object_exists(x))
        self.object = scene.add(
            DynamicCuboid(
                name=object_name,
                position=self.object_initial_position,
                orientation=self.object_initial_orientation,
                prim_path=object_prim_path,
                scale=self.object_size,
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        self._task_objects[self.object.name] = self.object
        self.robot = self.set_robot()
        scene.add(self.robot)
        self._task_objects[self.robot.name] = self.robot
        self._move_task_objects_to_their_frame()
        return
    
    @abstractmethod
    def set_robot(self):
        raise NotImplementedError

    def reset(self) -> None:
        self.place_position = None
        self.place_orientation = None
        self.target_position = None
        self.target_orientation = None
        self.object_in_hand = None

    def set_params(
        self,
        object_position: Optional[np.ndarray] = None,
        object_orientation: Optional[np.ndarray] = None,
    ) -> None:
        if object_position is not None or object_orientation is not None:
            self.object.set_local_pose(translation=object_position, orientation=object_orientation)
        return

    def get_params(self) -> dict:
        params_representation = dict()
        position, orientation = self.object.get_local_pose()
        params_representation["object_position"] = {"value": position, "modifiable": True}
        params_representation["object_orientation"] = {"value": orientation, "modifiable": True}
        params_representation["place_position"] = {"value": self.place_position, "modifiable": True}
        params_representation["place_orientation"] = {"value": self.place_orientation, "modifiable": True}
        params_representation["object_name"] = {"value": self.object.name, "modifiable": False}
        params_representation["robot_name"] = {"value": self.robot.name, "modifiable": False}
        return params_representation
    
    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self.robot.get_joints_state()
        object_position, object_orientation = self.object.get_local_pose()
        end_effector_position, end_effector_orientation = self.robot.end_effector.get_local_pose()
        return {
            self.object.name: {
                "position": object_position,
                "orientation": object_orientation,
            },
            self.robot.name: {
                "joint_positions": joints_state.positions,
                "end_effector_position": end_effector_position,
                "end_effector_orientation": end_effector_orientation,
            },
            "place_position": self.place_position,
            "place_orientation": self.place_orientation,
        }
    
    def set_target_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        self.target_position = position
        self.target_orientation = orientation

    def set_place_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        self.place_position = position
        self.place_orientation = orientation

    def apply_grasp_offset(self, translation: np.ndarray, orientation: np.ndarray) -> None:
        assert self.target_position is not None
        assert self.target_orientation is not None

        self.target_position += translation
        target_rotation_matrix = matmul(
            quats_to_rot_matrices(self.target_orientation),
            quats_to_rot_matrices(orientation)
        )
        self.target_orientation = rot_matrices_to_quats(target_rotation_matrix)


    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        return
    
    def post_reset(self) -> None:
        from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

        if isinstance(self.robot.gripper, ParallelGripper):
            self.robot.gripper.set_joint_positions(self.robot.gripper.joint_opened_positions)
        return
    
    def calculate_metrics(self) -> dict:
        """[summary]"""
        raise NotImplementedError

    def is_done(self) -> bool:
        """[summary]"""
        raise NotImplementedError
    