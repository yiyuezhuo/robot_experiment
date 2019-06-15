# Light weight Quadcopter drone bomber simulation 

The project is made for class assignment of robot system. 
It include some `*.urdf`, `.dae` texture with their blender python scripts(`/dae_blender_scripts`),
a simple dynamic simulation and a exciting Tiananmen scenario.


## Preview

<img src="preview/output2.webp">

## Install

```
git clone https://github.com/yiyuezhuo/robot_experiment.git
mv robot_experiment mydrone
cd mydrone
echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)" >> ~/.bashrc
source ~/.bashrc
roslaunch mydrone display_path.launch
```

## launch files

* `display.launch`: A drone model do rotional move.
* `display_grenade.launch`: A drone model do rotional move with a grenade.
* `display_control.launch`: A drone move to a target with PID controller repeatedly.
* `display_path.launch`: A drone move to a target with PID controller and draw the moving path.
* `display_path_noised.launch`: A drone move to a target with PID controller and noise.
* `display_path_noised_kalman.launch`: A drone move to a target with PID controller, noise and Kalman filter.
* `display_path_apf.launch`: A drone evade a tank using Artificial potential field(APF) method.
