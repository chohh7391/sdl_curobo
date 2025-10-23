import numpy as np
from typing import Optional

from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name

from sdl.tasks.base.cube_stack import BaseCubeStack
from sdl.robots.fr5 import FR5
from sdl.utils.util_file import join_path, get_assets_path


class CuroboCubeStackTask(BaseCubeStack):
    def __init__(
        self,
        name: str = "multi_modal_stacking",
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BaseCubeStack.__init__(
            self,
            name=name,
            cube_initial_positions=np.array(
                [
                    [0.50, 0.0, 0.1],
                    [0.50, -0.20, 0.1],
                    [0.50, 0.20, 0.1],
                    [0.30, -0.20, 0.1],
                    # [0.30, 0.0, 0.1],
                    [0.30, 0.20, 0.1],
                    [0.70, -0.20, 0.1],
                    # [0.70, 0.0, 0.1],
                    [0.70, 0.20, 0.1],
                ]
            )
            / get_stage_units(),
            cube_initial_orientations=None,
            stack_target_position=None,
            cube_size=np.array([0.045, 0.045, 0.07]),
            offset=offset,
        )
        self.cube_list = None
        self.target_position = None
        self.target_cube = None
        self.cube_in_hand = None

    def reset(self) -> None:
        self.cube_list = self.get_cube_names()
        self.target_position = None
        self.target_cube = None
        self.cube_in_hand = None

    def update_task(self) -> bool:
        # after detaching the cube in hand
        assert self.target_cube is not None
        assert self.cube_in_hand is not None
        self.cube_list.insert(0, self.cube_in_hand)
        self.target_cube = None
        self.target_position = None
        self.cube_in_hand = None
        if len(self.cube_list) <= 1:
            task_finished = True
        else:
            task_finished = False
        return task_finished

    def get_cube_prim(self, cube_name: str):
        for i in range(self._num_of_cubes):
            if cube_name == self._cubes[i].name:
                return self._cubes[i].prim_path

    def get_place_position(self, observations: dict) -> None:
        assert self.target_cube is not None
        self.cube_in_hand = self.target_cube
        self.target_cube = self.cube_list[0]
        ee_to_grasped_cube = (
            observations["my_fr5"]["end_effector_position"][2]
            - observations[self.cube_in_hand]["position"][2]
        )
        self.target_position = observations[self.target_cube]["position"] + [
            0,
            0,
            self._cube_size[2] + ee_to_grasped_cube,
        ]
        self.cube_list.remove(self.target_cube)

    def get_pick_position(self, observations: dict) -> None:
        assert self.cube_in_hand is None
        self.target_cube = self.cube_list[1]
        self.target_position = observations[self.target_cube]["position"] + [
            0,
            0,
            self._cube_size[2] / 2,
        ]
        self.cube_list.remove(self.target_cube)

    def set_robot(self) -> FR5:
        fr5_asset_path = join_path(get_assets_path(), "robot/dcp_description/usd/fr5_ag95/fr5_ag95.usd")
        fr5_prim_path = find_unique_string_name(
            initial_name="/World/FR5", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        fr5_robot_name = find_unique_string_name(
            initial_name="my_fr5", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        return FR5(
            prim_path=fr5_prim_path,
            name=fr5_robot_name,
            usd_path=fr5_asset_path,
            end_effector_prim_name="gripper_finger2_finger_tip_link",
            gripper_dof_names=["gripper_finger1_joint"],
            use_mimic_joints=True,
            gripper_open_position=np.array([0.0]),
            gripper_closed_position=np.array([0.6524]),
        )