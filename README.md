RL environment list
===================

A comprehensive list of environments for reinforcement learning categorized by
use. If there is an environment you think is missing, please submit a form
[here](https://github.com/clvrai/awesome-rl-envs/issues).

Each environment has a teaser figure next to it. Note all the teaser figures may take some
time to load.

Table of Contents
=================
* [Robotics](#robotics)
* [Games](#games)
* [Multi-Task Learning](#multi-task-learning)
* [Suites](#suites)
* [Navigation](#navigation)
* [Home (More Navigation)](#home-more-navigation)
* [Multi-Agent](#multi-agent)
* [Safety](#safety)
* [Autonomous Driving](#autonomous-driving)
* [Humanoid](#humanoid)
* [Text](#text)
* [Physics Simulators](#physics-simulators)
* [Misc](#misc)

## Robotics

| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/furn_change.gif' width=300 /> | [IKEA Furniture Assembly](https://clvrai.github.io/furniture/) for testing and accelerating the automation of complex long-horizon manipulation tasks including 80+ furniture models, 3 robots (Baxter, Sawyer, Cursor), and configurable backgrounds and textures. |
| <img src='media/doorgym.gif' width=300 /> | [DoorGym](https://github.com/PSVL/DoorGym) |
| <img src='media/play_data_small.gif' width=300 /> | [Playroom](https://github.com/google-research/google-research/tree/master/playrooms) |
| <img src='media/surreal.png' width=400 /> | [Robosuite](https://github.com/StanfordVL/robosuite) |
| <img src='media/roboschool.gif' width=300 /> | [Roboschool](https://openai.com/blog/roboschool/) |
| <img src='media/ml45-1080p.gif' width=500 /> | [Meta-World](https://github.com/rlworkgroup/metaworld) Includes 50 diverse robot manipulation tasks on a simulated Sawyer robotic arm. Also includes a variety of evaluation modes varying the number of training and testing tasks.   |
| <img src='media/rl-env.png' width=500 /> | [RLBench](https://sites.google.com/view/rlbench) |
| <img src='media/frs0_large.gif' width=300 /> | [RoboNet](https://www.robonet.wiki) |
| <img src='media/assistive_gym.jpg' width=300 /> | [Assistive-gym](https://github.com/Healthcare-Robotics/assistive-gym) 6 assistive tasks (ScratchItch, BedBathing, Feeding, Drinking, Dressing, and ArmManipulation).4 commercial robots (PR2, Jaco, Baxter, Sawyer). 2 human states: static or active (takes actions according to a separate control policy).Customizable female and male human models. 40 actuated human joints (head, torso, arms, waist, and legs).Realistic human joint limits|
| | [BairPushing Dataset](https://github.com/tensorflow/datasets/blob/master/tensorflow_datasets/video/bair_robot_pushing.py) |

## Games
| <img width=2000 />  | <img width=2000 /> |
| :---: | --- |
| <img src='media/pysc2.jpeg' width=300 /> | [StarCraft 2](https://github.com/deepmind/pysc2) |
| | [TorchCraft](https://github.com/TorchCraft/TorchCraft) |
| <img src='media/vizdom.gif' width=300 height=100 /> | [VizDoom](https://github.com/mwydmuch/ViZDoom) |
| <img src='media/fifa.gif' width=300 /> | [Soccer Simulator](https://github.com/google-research/football) |
| <img src='media/mine.gif' width=300 /> | [Minecraft](https://github.com/minerllabs/minerl) |
| <img src='media/phyre.gif' width=300 /> | [PHYRE](https://phyre.ai/) Benchmark for physical reasoning that contains a set of simple classical mechanics puzzles in a 2D enviroment | 
| <img src='media/marlo.gif' width=300 /> | [MarLÖ : Reinforcement Learning + Minecraft](https://github.com/crowdAI/marLo) A high level API built on top of Project MalmÖ to facilitate Reinforcement Learning experiments with a great degree of generalizability, capable of solving problems in pseudo-random, procedurally changing single and multi agent environments within the world of the mediatic phenomenon game Minecraft. |
| | [SuperMario](https://github.com/ppaquette/gym-super-mario) |
| <img src='media/gym_retro.gif' width=300 /> | [Gym Retro](https://github.com/openai/retro) |
| <img src='media/coin_runner.gif' width=300 /> | [Coin-Run](https://github.com/openai/coinrun) |


## Multi-Task Learning
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/ml45-1080p.gif' width=500 /> | [Meta-World](https://github.com/rlworkgroup/metaworld) Includes 50 diverse robot manipulation tasks on a simulated Sawyer robotic arm. Also includes a variety of evaluation modes varying the number of training and testing tasks.   |
| <img src='media/rl-env.png' width=500 /> | [RLBench](https://sites.google.com/view/rlbench) |
| <img src='media/play_data_small.gif' width=300 /> | [Playroom](https://github.com/google-research/google-research/tree/master/playrooms) |

## Suites
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/gym.gif' width=300 /> | [OpenAI Gym Robotics](https://github.com/openai/gym) | 
| <img src='media/gym-mujoco.gif' width=300 /> | [OpenAI Gym Mujoco](https://github.com/openai/gym) | 
| <img src='media/gym-classic.gif' width=300 /> | [OpenAI Gym Classic](https://github.com/openai/gym) | 
| <img src='media/gym-atari.gif' width=300 /> | [OpenAI Gym Atari](https://github.com/openai/gym) | 
| <img src='media/deepmind.png' width=500 /> | [DeepMind Control Suite](https://github.com/openai/gy://github.com/deepmind/dm_control) | 
| <img src='media/tennis.png' width=400 /> | [Unity Agents](https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md) |

## Navigation
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/obstacle.png' width=300 /> | [Obstacle Tower](https://github.com/Unity-Technologies/obstacle-tower-challenge) Traverse through procedurally generated floors which get progressively harder. Challenging visual inputs. |
| <img src='media/gym-maze.gif' width=300 /> | [gym-maze](https://github.com/MattChanTK/gym-maze) |
| <img src='media/multi-room.gif' width=300 /> | [gym-minigrid](https://github.com/maximecb/gym-minigrid) |
| <img src='media/miniworld.jpg' width=300 /> | [gym-miniworld](https://github.com/maximecb/gym-miniworld) |


## Home (More Navigation)
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/thor.gif' width=300 /> | [AI2THOR](https://ai2thor.allenai.org/) An Interactive 3D Environment for Visual AI |
| <img src='media/house3d.gif' width=300 /> | [House3D](https://github.com/facebookresearch/house3d) |
| <img src='media/home-platform.png' width=300 /> | [HoME: a Household Multimodal Environment](https://github.com/HoME-Platform/home-platform) platform for agents to learn from vision, audio, semantics, physics, and interaction with objects and other agents, all within a realistic context.|
| <img src='media/virtualhome.png' width=300 /> | [VirtualHome](http://virtual-home.org/) |
| <img src='media/isaac.jpg' width=300 /> | [Nvidia ISAAC simulator](https://developer.nvidia.com/Isaac-sdk) virtual robotics laboratory and a high-fidelity 3D world simulator |
| <img src='media/gibson.gif' width=300 /> | [Gibson](http://gibsonenv.stanford.edu/) 3d navigation in indoor scans |
| <img src='media/habitat_compressed.gif' width=300 /> | [Habitat](https://aihabitat.org) |
| <img src='media/minos.png' width=300 /> | [MINOS](https://minosworld.github.io/) |


## Multi-Agent
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/robosumo2.gif' width=300 /> | [RoboSumo](https://github.com/openai/robosumo) |
| <img src='media/multi_agent_particle_envs.gif' width=300 /> | [Multi-agent Particle Environment](https://github.com/openai/multiagent-particle-envs) A simple multi-agent particle world with a continuous observation and discrete action space, along with some basic simulated physics.|
| <img src='media/hideseek.png' width=300 /> | [OpenAI Multi-Agent Hide and Seek](https://github.com/openai/multi-agent-emergence-environments) |
| <img src='media/compete.png' width=300 /> | [OpenAI Multi-Agent Competition Environments](https://github.com/openai/multiagent-competition) |
| <img src='media/mmo.png' width=300 /> | [Massive Multi Agent Game Environment](https://github.com/openai/neural-mmo) |



## Safety
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/safety_gym.png' width=300 />  | [Safety Gym](https://github.com/openai/safety-gym) |
| <img src='media/assistive_gym.jpg' width=300 /> | [Assistive-gym](https://github.com/Healthcare-Robotics/assistive-gym) 6 assistive tasks (ScratchItch, BedBathing, Feeding, Drinking, Dressing, and ArmManipulation).4 commercial robots (PR2, Jaco, Baxter, Sawyer). 2 human states: static or active (takes actions according to a separate control policy).Customizable female and male human models. 40 actuated human joints (head, torso, arms, waist, and legs).Realistic human joint limits|
| | [DeepMind AI Safety Gridworlds ](https://github.com/deepmind/ai-safety-gridworlds) |



## Autonomous Driving
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/duckietown.png' width=300 /> | [DuckieTown](https://github.com/duckietown/gym-duckietown) |
| <img src='media/airsim.png' width=300 /> | [Autonomous Vehicle Simulator](https://github.com/Microsoft/AirSim) |
| <img src='media/deepdrive.gif' width=300 /> | [DeepDrive Self Driving Car Simulator](https://github.com/deepdrive/deepdrive) |
| <img src='media/streetlearn.gif' width=300 /> | [DeepMind StreetLearn](https://github.com/deepmind/streetlearn) |
| <img src='media/gta_drive.jpeg' width=300 /> | [DeepGTAV v2](https://github.com/aitorzip/DeepGTAV) |
| <img src='media/torcs.png' width=300 /> | [TORCS](https://sourceforge.net/projects/torcs/) |
| <img src='media/carla.gif' width=300 /> | [CARLA](http://carla.org/) |
| <img src='media/lgsvl.png' width=300 /> | [LGSVL](https://www.lgsvlsimulator.com/) |


## Humanoid
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/mass.png' width=500 /> | [Full Body Muscle Simulator](https://github.com/lsw9021/MASS) |
| <img src='media/running.gif' width=300 /> | [Osim-rl](http://osim-rl.stanford.edu/) Reinforcement learning environments with musculoskeletal models. Task: learning to walk/move/run using musculoskeletal models. |
| <img src='media/roboschool.gif' width=300 /> | [Roboschool](https://openai.com/blog/roboschool/) |



## Text
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| <img src='media/jericho.png' width=300 /> | [Jericho](https://github.com/microsoft/jericho) |

## Physics Simulators
| <img width=500 />  | <img width=8000 />  |
| :---: | --- |
| | [Mujoco-py](https://github.com/openai/mujoco-py) |
| | [PyBullet](https://pybullet.org/wordpress/) |
| | [DART](http://dartsim.github.io) |




## Misc
| <img width=8000 />  | <img width=8000 />  |
| :---: | --- |
| | [Reco Gym](https://github.com/criteo-research/reco-gym) Reinforcement Learning Environment for the problem of Product Recommendation in Online Advertising. |


