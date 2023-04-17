# Modules

## assets

This directory contains all definition files required to simulate, control and visualize
a vehicle.

## fcs

This is where the source code for a flight control sytem is.

## fdm

This directory contains an interface to JSBSim FDM. It provides a way to start the JSBSim
simulation, a way to hook a flight control system to the simulated vehicle, and a way to
stream the simulation data into a visualizer over websocket.

## j-viz

Small python script for real-time or static visualization of telemetry in a form of time series.

## visualizer

This program is a 3d visualizer intended to display real-time data in 3D using websocket interface.

# Required asset files

```
assets/
    aircraft/
        3d/
            <aircraft-name>.obj
            ...
        data_output/
            <aircraft-name>_out.xml
            ...
        output_def/
            <aircraft-name>_out_def.xml
            ...
        <aircraft-name>.xml
        <aircraft-name>_init_001.xml
        ...
    engine/
        <engine-name>.xml
        <properller-name>.xml
    script/
        <aircraft-name>_001.xml
        <aircraft-name>_002.xml
```

## .obj requirements

The craft, after loading, should have the nose to +x, up to -z and left -y. To achieve this, do the following in blender:

1. The craft has fwd pointing to -x, left is to -y, and down to -z
2. Export the craft with the following blender settings: +y forward, -z up
3. Don't "write materials"

# Python venv

```
$ python3 -m venv venv
$ source ./venv/bin/activate
(venv) $ pip install -r requirements.txt
```

# Feature requests

- Telemetry replay
- Use RTOS in fcs
- Improve pressure measurement using interrupts
- Try faster PWM duty cycle for better responses
- Remove external dependencies and allow running on a supercomputer
