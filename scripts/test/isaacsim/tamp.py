import os
from typing import Optional, List, Dict

from cutamp.algorithm import run_cutamp
from cutamp.config import TAMPConfiguration, validate_tamp_config
from cutamp.constraint_checker import ConstraintChecker
from cutamp.cost_reduction import CostReducer
from cutamp.envs import TAMPEnvironment
from cutamp.envs.book_shelf import load_book_shelf_env
from cutamp.envs.stick_button import load_stick_button_env
from cutamp.envs.tetris import load_tetris_env
from cutamp.envs.utils import get_env_dir, load_env

from cutamp.scripts.utils import (
    default_constraint_to_mult,
    default_constraint_to_tol,
    setup_logging,
    get_tetris_tuned_constraint_to_mult,
)
from cutamp.task_planning.base_structs import State
import logging
from cutamp.cost_reduction import CostReducer
from cutamp.envs.utils import TAMPEnvironment



def load_demo_env(name: str) -> TAMPEnvironment:
    if name.startswith("tetris_"):
        num_blocks = int(name.split("tetris_")[-1])
        env = load_tetris_env(num_blocks, buffer_multiplier=1.0)
    elif name == "book_shelf":
        env = load_book_shelf_env()
    elif name == "stick_button":
        env = load_stick_button_env()
    elif name == "blocks":
        env_path = os.path.join(get_env_dir(), "obstacle_blocks_large_region.yml")
        env = load_env(env_path)
    elif name == "unpack":
        env_path = os.path.join(get_env_dir(), "unpack_3.yml")
        env = load_env(env_path)
    else:
        raise ValueError(f"Unknown environment name: {name}")
    return env


class TAMP:

    def __init__(self):

        self.config = TAMPConfiguration(
            num_particles=1024,
            robot="panda",
            grasp_dof=4,
            approach="optimization",
            num_resampling_attempts=100,
            num_opt_steps=1000,
            max_loop_dur=None,
            optimize_soft_costs=False,
            soft_cost=None,
            num_initial_plans=30,
            cache_subgraphs=None,
            curobo_plan=True,
            enable_visualizer=True,
            opt_viz_interval=10,
            viz_robot_mesh=True,
            experiment_root="/tmp/cutamp-experiments",
        )
        validate_tamp_config(self.config)

        self.env = load_demo_env(name="tetris_3")

        setup_logging()

        self.use_tetris_tuned_weights = None

        self.constraint_to_mult = (
            get_tetris_tuned_constraint_to_mult() if self.use_tetris_tuned_weights else default_constraint_to_mult.copy()
        )
        self.cost_reducer = CostReducer(self.constraint_to_mult)
        self.constraint_checker = ConstraintChecker(default_constraint_to_tol.copy())
        self._log = logging.getLogger(__name__)

        self.curobo_plan = None
        self.total_num_satisfying = None

    def plan(
        self,
        q_init: Optional[List[float]] = None,
        experiment_id: Optional[str] = None
    ):
        self.curobo_plan, self.total_num_satisfying = run_cutamp(
            env=self.env,
            config=self.config,
            cost_reducer=self.cost_reducer,
            constraint_checker=self.constraint_checker,
            q_init=q_init,
            experiment_id=experiment_id
        )

        print(self.curobo_plan)

        return self.curobo_plan, self.total_num_satisfying
    
    def execute(self, curobo_plan: List[Dict]):
        
        """
        (arm_action)
        curobo_plan: [
            {
                "type": "trajectory"
                "plan": ,
                "dt": 
            }, ...
        ]

        (gripper_action)
        curobo_plan: [
            {
                "type": "gripper",
                "action": "close"
            }
        ]
        """
        pass

    def update_config(self, config: TAMPConfiguration):
        self.config = config

    def update_goal_state(self, goal_state: State):
        self.env.goal_state = goal_state

    def update_initial_state(self, initial_state: State):
        pass