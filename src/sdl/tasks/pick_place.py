import numpy as np
from typing import Optional

from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name

from sdl.tasks.base.pick_place import BasePickPlace
from sdl.robots.fr5 import FR5
from sdl.utils.util_file import join_path, get_assets_path


class CuroboPickPlace(BasePickPlace):
    def __init__(
        self,
        name: str = "curobo_pick_place",
        offset: Optional[np.ndarray] = None,
    ) -> None:
        BasePickPlace.__init__(self, name=name, offset=offset)

    def get_pick_pose(self, observations: dict) -> None:
        assert self.object_in_hand is None
        assert observations[self.object.name]["position"] is not None
        assert observations[self.object.name]["orientation"] is not None
        self.set_target_pose(
            position=observations[self.object.name]["position"],
            orientation=observations[self.object.name]["orientation"]
        )
        self.apply_grasp_offset(
            translation=np.array([0, 0, 0.02]),
            orientation=np.array([0, 0, 1, 0]),
        )

    def get_place_pose(self, observations: dict) -> None:
        assert observations["place_position"] is not None
        assert observations["place_orientation"] is not None
        self.object_in_hand = self.object
        self.set_target_pose(
            position=observations["place_position"],
            orientation=observations["place_orientation"]
        )
        self.apply_grasp_offset(
            translation=np.array([0, 0, 0.02]),
            orientation=np.array([0, 0, 1, 0]),
        )
        
    def update_task(self) -> None:
        # after detaching the cube in hand
        assert self.object_in_hand is not None
        self.target_position = None
        self.object_in_hand = None
        self.set_params(
            object_position=np.array([0.5, 0.0, self.object_size[2] / 2]),
            object_orientation=np.array([1.0, 0.0, 0.0, 0.0])
        )

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