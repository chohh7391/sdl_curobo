# Standard Library
import numpy as np

# Third Party
import carb
from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.tasks import Stacking as BaseStacking
from isaacsim.core.utils.types import ArticulationAction

# CuRobo
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.geom.sphere_fit import SphereFitType
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.types.state import JointState
from curobo.util.usd_helper import UsdHelper
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
    MotionGenResult,
    PoseCostMetric,
)

from sdl.utils.util_file import get_robot_configs_path, get_world_configs_path, join_path, load_yaml



class CuroboCubeStackController(BaseController):
    def __init__(
        self,
        my_world: World,
        my_task: BaseStacking,
        name: str = "curobo_cube_stack_controller",
        constrain_grasp_approach: bool = False,
    ) -> None:
        BaseController.__init__(self, name=name)
        self._save_log = False
        self.my_world = my_world
        self.my_task = my_task
        self._step_idx = 0
        n_obstacle_cuboids = 20
        n_obstacle_mesh = 2
        # warmup curobo instance
        self.usd_help = UsdHelper()
        self.init_curobo = False
        self.world_file = "collision_table.yml"
        self.cmd_js_names = [
            "j1",
            "j2",
            "j3",
            "j4",
            "j5",
            "j6",
        ]
        self.tensor_args = TensorDeviceType()
        self.robot_cfg = load_yaml(join_path(get_robot_configs_path(), "fr5_ag95.yml"))["robot_cfg"]
        self.robot_cfg["kinematics"][
            "base_link"
        ] = "base_link"  # controls which frame the controller is controlling

        self.robot_cfg["kinematics"][
            "ee_link"
        ] = "grasp_frame"  # controls which frame the controller is controlling
        # self.robot_cfg["kinematics"]["cspace"]["max_acceleration"] = 10.0 # controls how fast robot moves
        self.robot_cfg["kinematics"]["extra_collision_spheres"] = {"attached_object": 100}
        # @self.robot_cfg["kinematics"]["collision_sphere_buffer"] = 0.0
        self.robot_cfg["kinematics"]["collision_spheres"] = "spheres/fr5_ag95.yml"

        world_cfg_table = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        )
        self._world_cfg_table = world_cfg_table

        world_cfg1 = WorldConfig.from_dict(
            load_yaml(join_path(get_world_configs_path(), "collision_table.yml"))
        ).get_mesh_world()
        world_cfg1.mesh[0].pose[2] = -10.5

        self._world_cfg = WorldConfig(cuboid=world_cfg_table.cuboid, mesh=world_cfg1.mesh)

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.robot_cfg,
            self._world_cfg,
            self.tensor_args,
            trajopt_tsteps=32,
            collision_checker_type=CollisionCheckerType.MESH,
            use_cuda_graph=True,
            interpolation_dt=0.01,
            collision_cache={"obb": n_obstacle_cuboids, "mesh": n_obstacle_mesh},
            store_ik_debug=self._save_log,
            store_trajopt_debug=self._save_log,
        )
        self.motion_gen = MotionGen(motion_gen_config)
        print("warming up...")
        self.motion_gen.warmup(parallel_finetune=True)
        pose_metric = None
        if constrain_grasp_approach:
            pose_metric = PoseCostMetric.create_grasp_approach_metric(
                offset_position=0.1, tstep_fraction=0.8
            )

        self.plan_config = MotionGenPlanConfig(
            enable_graph=True,
            max_attempts=10,
            enable_graph_attempt=None,
            enable_finetune_trajopt=True,
            partial_ik_opt=False,
            parallel_finetune=True,
            pose_cost_metric=pose_metric,
            time_dilation_factor=0.75,
        )
        self.usd_help.load_stage(self.my_world.stage)
        self.cmd_plan = None
        self.cmd_idx = 0
        self._step_idx = 0
        self.idx_list = None

    def attach_obj(
        self,
        sim_js: JointState,
        js_names: list,
    ) -> None:
        cube_name = self.my_task.get_cube_prim(self.my_task.target_cube)

        cu_js = JointState(
            position=self.tensor_args.to_device(sim_js.positions),
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=js_names,
        )

        self.motion_gen.attach_objects_to_robot(
            cu_js,
            [cube_name],
            sphere_fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE,
            world_objects_pose_offset=Pose.from_list([0, 0, 0.01, 1, 0, 0, 0], self.tensor_args),
        )

    def detach_obj(self) -> None:
        self.motion_gen.detach_object_from_robot()

    def plan(
        self,
        ee_translation_goal: np.array,
        ee_orientation_goal: np.array,
        sim_js: JointState,
        js_names: list,
    ) -> MotionGenResult:
        ik_goal = Pose(
            position=self.tensor_args.to_device(ee_translation_goal),
            quaternion=self.tensor_args.to_device(ee_orientation_goal),
        )
        cu_js = JointState(
            position=self.tensor_args.to_device(sim_js.positions),
            velocity=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            acceleration=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            jerk=self.tensor_args.to_device(sim_js.velocities) * 0.0,
            joint_names=js_names,
        )
        cu_js = cu_js.get_ordered_joint_state(self.motion_gen.kinematics.joint_names)
        result = self.motion_gen.plan_single(cu_js.unsqueeze(0), ik_goal, self.plan_config.clone())
        if self._save_log:  # and not result.success.item(): # logging for debugging
            UsdHelper.write_motion_gen_log(
                result,
                {"robot_cfg": self.robot_cfg},
                self._world_cfg,
                cu_js,
                ik_goal,
                join_path("log/usd/", "cube") + "_debug",
                write_ik=False,
                write_trajopt=True,
                visualize_robot_spheres=True,
                link_spheres=self.motion_gen.kinematics.kinematics_config.link_spheres,
                grid_space=2,
                write_robot_usd_path="log/usd/assets",
            )
        return result

    def forward(
        self,
        sim_js: JointState,
        js_names: list,
    ) -> ArticulationAction:
        assert self.my_task.target_position is not None
        assert self.my_task.target_cube is not None

        if self.cmd_plan is None:
            self.cmd_idx = 0
            self._step_idx = 0
            # Set EE goals
            ee_translation_goal = self.my_task.target_position
            ee_orientation_goal = np.array([0, 0, 1, 0])
            # compute curobo solution:
            result = self.plan(ee_translation_goal, ee_orientation_goal, sim_js, js_names)
            succ = result.success.item()
            if succ:
                cmd_plan = result.get_interpolated_plan()
                self.idx_list = [i for i in range(len(self.cmd_js_names))]
                self.cmd_plan = cmd_plan.get_ordered_joint_state(self.cmd_js_names)
            else:
                carb.log_warn("Plan did not converge to a solution.")
                return None
        if self._step_idx % 3 == 0:
            cmd_state = self.cmd_plan[self.cmd_idx]
            self.cmd_idx += 1

            # get full dof state
            art_action = ArticulationAction(
                cmd_state.position.cpu().numpy(),
                cmd_state.velocity.cpu().numpy() * 0.0,
                joint_indices=self.idx_list,
            )
            if self.cmd_idx >= len(self.cmd_plan.position):
                self.cmd_idx = 0
                self.cmd_plan = None
        else:
            art_action = None
        self._step_idx += 1
        return art_action

    def reached_target(self, observations: dict) -> bool:
        curr_ee_position = observations["my_fr5"]["end_effector_position"]
        if np.linalg.norm(
            self.my_task.target_position - curr_ee_position
        ) < 0.09 and (  # This is half gripper width, curobo succ threshold is 0.5 cm
            self.cmd_plan is None
        ):
            if self.my_task.cube_in_hand is None:
                print("reached picking target: ", self.my_task.target_cube)
            else:
                print("reached placing target: ", self.my_task.target_cube)
            return True
        else:
            return False

    def reset(
        self,
        ignore_substring: str,
        robot_prim_path: str,
    ) -> None:
        # init
        self.update(ignore_substring, robot_prim_path)
        self.init_curobo = True
        self.cmd_plan = None
        self.cmd_idx = 0

    def update(
        self,
        ignore_substring: str,
        robot_prim_path: str,
    ) -> None:
        # print("updating world...")
        obstacles = self.usd_help.get_obstacles_from_stage(
            ignore_substring=ignore_substring, reference_prim_path=robot_prim_path
        ).get_collision_check_world()
        # add ground plane as it's not readable:
        obstacles.add_obstacle(self._world_cfg_table.cuboid[0])
        self.motion_gen.update_world(obstacles)
        self._world_cfg = obstacles