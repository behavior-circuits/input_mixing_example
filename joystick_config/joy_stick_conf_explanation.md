This launch file configures the joystick to output a to the topic /joy_cmd
Optimally this shoudl be done using a remap in the launch file of this project.
Up until then this launch file can be used to start the /teleop_twist_joy package with the correct configuration.
To use it simply copy the launch file in the directory of the package (roscd teleop_twist_joy/launch)

The joystick can then be started using roslaunch teleop_twist_joy teleop_circuit.launch

