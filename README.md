A comprehensive list of environments for reinforcement learning categorized by
use. If there is an environment you think is missing, please submit a form
[here](www.google.com).

Each environment has a teaser figure next to it. Note all the teaser figures may take some
time to load.

## Robotics
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/furn.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://clvrai.github.io/furniture/'>IKEA Furniture Assembly</a> 
        <ul>
          <li>
            Furniture assembly. Complex long-horizon manipulation task.
          </li> 
          <li>
            Diversity. 80+ furniture models, customizable background, lighting
            and textures
          </li> 
          <li>
            Robots. Baxter, Sawyer, and more
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/doorgym.gif' width=300 />
      </td>
      <td width='50%'> <a href='https://github.com/PSVL/DoorGym'>DoorGym</a>
        <ul>
          <li>
            Train a policy to open up various doors. 
          </li> 
          <li>
            Unity integration
          </li> 
          <li>
            Random door knob generator and door knob dataset.
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/play_data_small.gif' width=300 />
      </td>
      <td width='50%'> <a href='https://github.com/google-research/google-research/tree/master/playrooms'>Playroom</a>
        <ul>
          <li>
            Variety of tasks in desk scenario. 
          </li> 
          <li>
            Evaluation code and play dataset will be included soon. 
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/surreal.png' width=400 />
      </td>
      <td width='50%'> <a href='https://github.com/StanfordVL/robosuite'>Robosuite</a>
      <ul>
        <li>
          A set of standard benchmarking tasks in robots. 
        </li> 
        <li>
          Defines a framework for easily creating new tasks and environments.
        </li> 
      </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/roboschool.gif' width=300 />
      </td>
      <td width='50%'> <a href='https://openai.com/blog/roboschool/'>Roboschool</a>
      <ul>
        <li>
          Control robots in simulation. 
        </li> 
        <li>
          Can use other physics engines other than MuJoCo.
        </li> 
        <li>
          Alternative to standard OpenAI Gym mujoco environments.
        </li> 
        <li>
          Easy to train multiple agents at once.
        </li> 
      </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/ml45-1080p.gif' width=500 />
      </td>
      <td width='50%'>
        <a href='https://github.com/rlworkgroup/metaworld'>Meta-World</a> 
        <ul>
          <li>
            50 diverse robot manipulation tasks on a simulated Sawyer robotic arm
          </li> 
          <li>
            Also includes a variety of evaluation modes varying the number of training and testing tasks.   
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/rl-env.png' width=500 />
      </td>
      <td width='50%'>
        <a href='https://sites.google.com/view/rlbench'>RLBench</a>
        <ul>
          <li>
            100 unique, hand designed tasks.
          </li> 
          <li>
            Vision-guided manipulation, imitation learning, multi-task
            learning, geometric computer vision and few-shot learning. 
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/frs0_large.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://www.robonet.wiki'>RoboNet</a>
        <ul>
          <li>
            A dataset for sharing robot experience including actions and frames
            from robots performing various tasks.
          </li> 
          <li>
            15 million video frames from 113 unique camera viewpoints
          </li> 
          <li>
            7 robot platforms
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/assistive_gym.jpg' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/Healthcare-Robotics/assistive-gym'> Assistive-gym </a> 
        <ul>
          <li>
           6 assistive tasks (ScratchItch, BedBathing, Feeding, Drinking, Dressing, and ArmManipulation).   
          </li> 
          <li>
           4 commercial robots (PR2, Jaco, Baxter, Sawyer).   
          </li> 
          <li>
            2 human states: static or active (takes actions according to a separate control policy).  
          </li> 
          <li>
           Customizable female and male human models. 40 actuated human joints (head, torso, arms, waist, and legs).Realistic human joint limit.
          </li> 
        </ul> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
      </td>
      <td width='50%'>
        <a href='https://github.com/tensorflow/datasets/blob/master/tensorflow_datasets/video/bair_robot_pushing.py'>BairPushing Dataset</a>
        <ul>
          <li>
            Dataset of Sawyer pushing objects. 
          </li> 
        </ul> 
      </td>
    </tr>
  </tbody>
</table>

## Games
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/pysc2.jpeg' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/deepmind/pysc2'>StarCraft 2</a>
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
      </td>
      <td width='50%'>
        <a href='https://github.com/TorchCraft/TorchCraft'>TorchCraft</a>
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/vizdom.gif' width=300 height=100 />
      </td>
      <td width='50%'>
        <a href='https://github.com/mwydmuch/ViZDoom'>VizDoom</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/fifa.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/google-research/football'>Soccer Simulator</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/mine.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/minerllabs/minerl'>Minecraft</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/phyre.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://phyre.ai/'>PHYRE</a> Benchmark for physical reasoning that contains a set of simple classical mechanics puzzles in a 2D enviroment 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/marlo.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/crowdAI/marLo'>MarLÖ : Reinforcement Learning + Minecraft</a> A high level API built on top of Project MalmÖ to facilitate Reinforcement Learning experiments with a great degree of generalizability, capable of solving problems in pseudo-random, procedurally changing single and multi agent environments within the world of the mediatic phenomenon game Minecraft. 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
      </td>
      <td width='50%'>
        <a href='https://github.com/ppaquette/gym-super-mario'>SuperMario</a>
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
      </td>
      <td width='50%'>
        <a href='https://github.com/openai/retro'>Gym Retro</a>
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/coin_runner.gif' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/openai/coinrun'>Coin-Run</a> 
      </td>
    </tr>
  </tbody>
</table>


## Multi-Task Learning
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/ml45-1080p.gif' width=500 />
      </td>
      <td>
        <a href='https://github.com/rlworkgroup/metaworld'>Meta-World</a> Includes 50 diverse robot manipulation tasks on a simulated Sawyer robotic arm. Also includes a variety of evaluation modes varying the number of training and testing tasks.   
      </td>
    </tr>
      <tr>
        <td width='50%' align='center'>
        <img src='media/rl-env.png' width=500 />
        </td>
        <td>
        <a href='https://sites.google.com/view/rlbench'>RLBench</a> 
        </td>
      </tr>
        <tr>
          <td width='50%' align='center'>
        <img src='media/play_data_small.gif' width=300 />
          </td>
          <td>
        <a href='https://github.com/google-research/google-research/tree/master/playrooms'>Playroom</a> 
          </td>
        </tr>
  </tbody>
</table>

## Suites
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/gym.gif' width=300 />
      </td>
      <td>
        <a href='https://github.com/openai/gym'>OpenAI Gym Robotics</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
      <img src='media/gym-mujoco.gif' width=300 />
      </td>
      <td>
        <a href='https://github.com/openai/gym'>OpenAI Gym Mujoco</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/gym-classic.gif' width=300 />
      </td>
      <td>
        <a href='https://github.com/openai/gym'>OpenAI Gym Classic</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/gym-atari.gif' width=300 />
      </td>
      <td>
        <a href='https://github.com/openai/gym'>OpenAI Gym Atari</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/deepmind.png' width=500 />
      </td>
      <td>
        <a href='https://github.com/openai/gy://github.com/deepmind/dm_control'>DeepMind Control Suite</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
        <img src='media/tennis.png' width=400 />
      </td>
      <td>
        <a href='https://github.com/Unity-Technologies/ml-agents/blob/master/docs/Learning-Environment-Examples.md'>Unity Agents</a> 
      </td>
    </tr>
  </tbody>
</table>

## Navigation
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/obstacle.png' width=300 />
      </td>
      <td>
        <a href='https://github.com/Unity-Technologies/obstacle-tower-challenge'>Obstacle Tower</a> Traverse through procedurally generated floors which get progressively harder. Challenging visual inputs. 
      </td>
    </tr>
      <tr>
        <td width='50%' align='center'>
        <img src='media/gym-maze.gif' width=300 />
        </td>
        <td>
        <a href='https://github.com/MattChanTK/gym-maze'>gym-maze</a> 
        </td>
      </tr>
        <tr>
          <td width='50%' align='center'>
        <img src='media/multi-room.gif' width=300 />
          </td>
          <td>
        <a href='https://github.com/maximecb/gym-minigrid'>gym-minigrid</a> 
          </td>
        </tr>
          <tr>
            <td width='50%' align='center'>
        <img src='media/miniworld.jpg' width=300 />
            </td>
            <td>
        <a href='https://github.com/maximecb/gym-miniworld'>gym-miniworld</a> 
            </td>
          </tr>
  </tbody>
</table>


## Home (More Navigation)
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/thor.gif' width=300 />
      </td>
      <td>
        <a href='https://ai2thor.allenai.org/'>AI2THOR</a> An Interactive 3D Environment for Visual AI 
      </td>
    </tr>
      <tr>
        <td width='50%' align='center'>
        <img src='media/house3d.gif' width=300 />
        </td>
        <td>
        <a href='https://github.com/facebookresearch/house3d'>House3D</a> 
        </td>
      </tr>
        <tr>
          <td width='50%' align='center'>
        <img src='media/home-platform.png' width=300 />
          </td>
          <td>
        <a href='https://github.com/HoME-Platform/home-platform'>HoME: a Household Multimodal Environment</a> platform for agents to learn from vision, audio, semantics, physics, and interaction with objects and other agents, all within a realistic context.
          </td>
        </tr>
          <tr>
            <td width='50%' align='center'>
        <img src='media/virtualhome.png' width=300 />
            </td>
            <td>
        [VirtualHome](http://virtual-home.org/) 
            </td>
          </tr>
            <tr>
              <td width='50%' align='center'>
        <img src='media/isaac.jpg' width=300 />
              </td>
              <td>
        <a href='https://developer.nvidia.com/Isaac-sdk'>Nvidia ISAAC simulator</a> virtual robotics laboratory and a high-fidelity 3D world simulator 
              </td>
            </tr>
              <tr>
                <td width='50%' align='center'>
        <img src='media/gibson.gif' width=300 />
                </td>
                <td>
        [Gibson](http://gibsonenv.stanford.edu/) 3d navigation in indoor scans 
                </td>
              </tr>
                <tr>
                  <td width='50%' align='center'>
        <img src='media/habitat_compressed.gif' width=300 />
                  </td>
                  <td>
        <a href='https://aihabitat.org'>Habitat</a> 
                  </td>
                </tr>
                  <tr>
                    <td width='50%' align='center'>
        <img src='media/minos.png' width=300 />
                    </td>
                    <td>
        <a href='https://minosworld.github.io/'>MINOS</a> 
                    </td>
                  </tr>
  </tbody>
</table>


## Multi-Agent
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/robosumo2.gif' width=300 />
      </td>
      <td>
        <a href='https://github.com/openai/robosumo'>RoboSumo</a> 
      </td>
    </tr>
    <tr>
      <td width='50%' align='center'>
      </td>
      <td width='50%'>
        <a href='https://github.com/openai/multiagent-particle-envs'>Multi-agent Particle Environment</a> A simple multi-agent particle world with a continuous observation and discrete action space, along with some basic simulated physics
      </td>
    </tr>
      <tr>
        <td width='50%' align='center'>
        <img src='media/hideseek.png' width=300 />
        </td>
        <td>
        <a href='https://github.com/openai/multi-agent-emergence-environments'>OpenAI Multi-Agent Hide and Seek</a> 
        </td>
      </tr>
        <tr>
          <td width='50%' align='center'>
        <img src='media/compete.png' width=300 />
          </td>
          <td>
        <a href='https://github.com/openai/multiagent-competition'>OpenAI Multi-Agent Competition Environments</a> 
          </td>
        </tr>
          <tr>
            <td width='50%' align='center'>
        <img src='media/mmo.png' width=300 />
            </td>
            <td>
        <a href='https://github.com/openai/neural-mmo'>Massive Multi Agent Game Environment</a> 
            </td>
          </tr>
  </tbody>
</table>



## Safety
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/safety_gym.png' width=300 />
      </td>
      <td>
        <a href='https://github.com/openai/safety-gym'>Safety Gym</a> 
      </td>
    </tr>
      <tr>
        <td width='50%' align='center'>
        <img src='media/assistive_gym.jpg' width=300 />
        </td>
        <td>
        <a href='https://github.com/Healthcare-Robotics/assistive-gym'>Assistive-gym</a> 6 assistive tasks (ScratchItch, BedBathing, Feeding, Drinking, Dressing, and ArmManipulation).4 commercial robots (PR2, Jaco, Baxter, Sawyer). 2 human states: static or active (takes actions according to a separate control policy).Customizable female and male human models. 40 actuated human joints (head, torso, arms, waist, and legs).Realistic human joint limits
        </td>
      </tr>
      <tr>
        <td width='50%' align='center'>
        </td>
        <td width='50%'>
        <a href='https://github.com/deepmind/ai-safety-gridworlds'>DeepMind AI Safety Gridworlds </a>
        </td>
      </tr>
  </tbody>
</table>



## Autonomous Driving
<table>
  <tbody>
  <tr>
    <td width='50%' align='center'>
      <img src='media/duckietown.png' width=300 />
    </td>
    <td width='50%'>
      <a href='https://github.com/duckietown/gym-duckietown'>DuckieTown</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/airsim.png' width=300 />
    </td>
    <td width='50%'>
      <a href='https://github.com/Microsoft/AirSim'>Autonomous Vehicle Simulator</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/deepdrive.gif' width=300 />
    </td>
    <td width='50%'>
      <a href='https://github.com/deepdrive/deepdrive'>DeepDrive Self Driving Car Simulator</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/streetlearn.gif' width=300 />
    </td>
    <td width='50%'>
      <a href='https://github.com/deepmind/streetlearn'>DeepMind StreetLearn</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/gta_drive.jpeg' width=300 />
    </td>
    <td width='50%'>
      <a href='https://github.com/aitorzip/DeepGTAV'>DeepGTAV v2</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/torcs.png' width=300 />
    </td>
    <td width='50%'>
      <a href='https://sourceforge.net/projects/torcs/'>TORCS</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/carla.gif' width=300 />
    </td>
    <td width='50%'>
      <a href='http://carla.org/'>CARLA</a> 
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
      <img src='media/lgsvl.png' width=300 />
    </td>
    <td width='50%'>
      <a href='https://www.lgsvlsimulator.com/'>LGSVL</a> 
    </td>
  </tr>
  </tbody>
</table>


## Humanoid
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/mass.png' width=500 />
      </td>
      <td>
        <a href='https://github.com/lsw9021/MASS'>Full Body Muscle Simulator</a> 
      </td>
    </tr>
      <tr>
        <td width='50%' align='center'>
        <img src='media/running.gif' width=300 />
        </td>
        <td>
        [Osim-rl](http://osim-rl.stanford.edu/) Reinforcement learning environments with musculoskeletal models. Task: learning to walk/move/run using musculoskeletal models. 
        </td>
      </tr>
        <tr>
          <td width='50%' align='center'>
        <img src='media/roboschool.gif' width=300 />
          </td>
          <td>
        <a href='https://openai.com/blog/roboschool/'>Roboschool</a> 
          </td>
        </tr>
  </tbody>
</table>



## Text
<table>
  <tbody>
    <tr>
      <td width='50%' align='center'>
        <img src='media/jericho.png' width=300 />
      </td>
      <td width='50%'>
        <a href='https://github.com/microsoft/jericho'>Jericho</a> 
      </td>
    </tr>
  </tbody>
</table>

## Physics Simulators
<table>
  <tbody>
  <tr>
    <td width='50%' align='center'>
    </td>
    <td width='50%'>
        <a href='https://github.com/openai/mujoco-py'>Mujoco-py</a>
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
    </td>
    <td width='50%'>
        <a href='https://pybullet.org/wordpress/'>PyBullet</a>
    </td>
  </tr>
  <tr>
    <td width='50%' align='center'>
    </td>
    <td width='50%'>
        <a href='http://dartsim.github.io'>DART</a>
    </td>
  </tr>
  </tbody>
</table>




## Misc
<table>
  <tbody>
  <tr>
    <td width='50%' align='center'>
    </td>
    <td width='50%'>
        <a href='https://github.com/criteo-research/reco-gym'>Reco Gym</a> Reinforcement Learning Environment for the problem of Product Recommendation in Online Advertising.
    </td>
  </tr>
  </tbody>
</table>


