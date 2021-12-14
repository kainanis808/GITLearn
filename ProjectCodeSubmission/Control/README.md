Add your code description to this README.md file.
This is a README for the controller portion of the GNC scheme.

## Inputs
Operator should call upon the script and pass arguments to redefine PID gains. The format follows:

> controllerReset.py *P<sub>fwd_vel</sub> I<sub>fwd_vel</sub> D<sub>fwd_vel</sub> P<sub>yaw_rate</sub> I<sub>yaw_rate</sub> D<sub>yaw_rate</sub>*

An example is shown below:
> controllerReset.py 40 2 20 60 0 20

In this example the gains are distributed such that the forward velocity PID gains are P = 40, I = 2, and D = 20. Similarly, the yaw rate PID gains are P = 60, I = 0, and D = 20.

## Outputs
- Redefine parameters of in-built PID gains for forward velocity and yaw raw. This is indicated to operators via a print statement in the terminal that calls the script.
- Reinstantiate the Gazebo world with new PID gains. This is indicated to operators via a print statement in the terminal that calls the script and the Gazebo simulation situates the Heron at the world origin.

## Dependancies
None

## Etc.
This script utilizes the pre-existing Heron Controller from Clearpath Robotics. The GITHUB repository for the Heron controller can be accessed [here](https://github.com/heron/heron_controller.git). 
Users that are unfamiliar with using the Heron and ROS should follow the guide provided by Clearpath. The official guide for installing and simulating the Heron in ROS Melodic can be accessed [here](https://www.clearpathrobotics.com/assets/guides/melodic/heron/simulation.html). In accordance with recommended Ubuntu/ROS file hierarchy, it is recommended to place the controllerReset.py file into the scripts folder of the Heron Controller such that the directory is similar to the following path:
> user@user: ~/workspace/src/heron_simulator/heron_gazebo/scripts
