# behavior_circuit_agent
Implementation of a autonomous Agent based on behavior circuits



## Starting the Turtlebot
To start the turtlebot one has to remote ssh into the raspberry pi.
Additionally one should synchronize the clocks between host and robot so that the navigationstack works.
Both can be done at the same time by calling:

```
ssh pi@10.0.0.250 sudo date -s @$(date -u +"%s")
```

## Starting a Circuit
All available circuits are defined in the `scripts/circuits.py` and can be launched using:
```
roslaunch behaviour_circuits circuit.launch circuit:=CIRCUIT_NAME
```
Where `CIRCUIT_NAME` is the name of the desired circuit as defined in `scripts/circuits.py`.
