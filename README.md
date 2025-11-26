# SDL - curobo

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)

## Overview

This project utilizes curobo within Isaac Sim to perform simple **pick and place** & **cube stack**

## Installation

- Create conda environment

  ```
  conda create -n sdl_curobo python=3.11
  conda activate sdl_curobo
  ```

- Install dependencies
  
  ```bash
  pip install --upgrade pip
  pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
  pip install torch==2.7.0 torchvision==0.22.0 torchaudio --index-url https://download.pytorch.org/whl/cu128

  cd ~/sdl_curobo/src
  git clone https://github.com/NVlabs/curobo.git
  python -m pip install tomli wheel ninja

  cd curobo
  # This can take up to 20 minutes to install
  python -m pip install -e .[isaacsim] --no-build-isolation

  cd ~/sdl_curobo
  pip install -e .
  ```

## Getting Started

  - curobo with isaacsim
  
  ```
  conda activate sdl_curobo
  python scripts/pick_and_place.py
  ```

  ```
  conda activate sdl_curobo
  python scripts/cube_stacking.py
  ```
  
