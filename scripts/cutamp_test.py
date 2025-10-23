import os
from typing import Optional

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


if __name__ == "__main__":
    
    config = TAMPConfiguration(
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
    validate_tamp_config(config)

    env = load_demo_env(name="tetris_3")

    # cutamp demo
    setup_logging()

    use_tetris_tuned_weights = None

    constraint_to_mult = (
        get_tetris_tuned_constraint_to_mult() if use_tetris_tuned_weights else default_constraint_to_mult.copy()
    )
    cost_reducer = CostReducer(constraint_to_mult)
    constraint_checker = ConstraintChecker(default_constraint_to_tol.copy())

    run_cutamp(env, config, cost_reducer, constraint_checker, experiment_id=use_tetris_tuned_weights)