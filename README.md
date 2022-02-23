# Input Mixing Example

This repository contains example code for shared control between driver and assistive systems using input mixing based on behavior circuits.

The code runs on ROS melodic or higher and uses a Turtlebot burger allthough it can be configured for arbitrary robots with acces to the ROS navigation stack.

## Assistance Systems
The System features one emergency breaking systems as well as two steering assistance modeled by the navigation stack and a circuit based homing.
These are combined with the human input using a behavior circuit.



## Starting a Circuit
All available circuits are defined in the `scripts/circuits.py` and can be launched using:
```
roslaunch behaviour_circuits circuit.launch circuit:=CIRCUIT_NAME
```
Where `CIRCUIT_NAME` is the name of the desired circuit as defined in `scripts/circuits.py`.
This will launch every needed assistance system as well as the circuit itself.

In order to run the experiments described on the [behavior circuit website](https://behavior-circuits.github.io/website/examples/input_mixing/) one has to run:
```
roslaunch behaviour_circuits experiment_1.launch
roslaunch behaviour_circuits experiment_3.launch
roslaunch behaviour_circuits experiment_5.launch
```
respectively for the baseline experiment, impeded joystick experiment and assisted experiment.
