# SDL - Isaac Sim

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.0.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)

## Overview

This project utilizes cuTAMP within Isaac Sim to perform **S**elf-**D**riving **L**aboratories

## Installation

- Create conda environment

  ```
  conda create -n sdl_isaacsim python=3.11
  conda activate sdl_isaacsim
  ```

- Installation of dependencies
  
  - Install a CUDA-enabled PyTorch 2.7.0 build for CUDA 12.8:
  ```
  pip install torch==2.7.0 torchvision==0.22.0 torchaudio --index-url https://download.pytorch.org/whl/cu128
  ```

  - Install Isaac Sim pip packages:

  ```
  pip install "isaacsim[all,extscache]==5.0.0" --extra-index-url https://pypi.nvidia.com
  ```

  - git clone repository
  ```
  cd ~/
  git clone https://github.com/chohh7391/sdl_isaacsim.git
  ```

  - Install cutamp dependencies
  ```
  cd ~/sdl_isaacsim/src/cuTAMP
  pip install -e .

  sudo apt install git-lfs
  git lfs install
  ```

  - Install curobo dependencies
  ```
  cd ~/sdl_isaacsim/src/curobo
  python -m pip install tomli wheel ninja
  
  # This can take up to 20 minutes to install
  python -m pip install -e .[isaacsim] --no-build-isolation
  ```

## Getting Started

  - cutamp demo
  ```
  conda activate sdl_isaacsim
  cutamp-demo
  ```

  - isaacsim with curobo
  ```
  conda activate sdl_isaacsim
  python scripts/pick_and_place.py
  ```

  ```
  conda activate sdl_isaacsim
  python scripts/cube_stacking.py
  ```
  
