# Modules

## assets

This directory contains all definition files required to simulate, control and visualize
a vehicle.

## fcs

This directory is where the source code for a flight control sytem is.

## fdm

This directory contains an interface to JSBSim FDM. It provides a way to start the JSBSim
simulation, a way to hook a flight control system to the simulated vehicle, and a way to
stream the simulation data into a visualizer over websocket.

## visualizer

This program is a 3d visualizer intended to run in a browser to display real-time data.

# Required files

```
assets/
    aircraft/
        <aircraft-name>.xml
        <!-- <aircraft-name>_fcs.xml -->
        <!-- <aircraft-name>_viz.xml -->
        <aircraft-name>.obj
        <aircraft-name>_init_001.xml
    engine/
        <engine-name>.xml
    script/
        <aircraft-name>_001.xml
        <aircraft-name>_002.xml
```

## .obj requirements

The craft, after loading, should have the nose to +x, up to -z and left -y. To achieve this, do the following in blender:

1. The craft has fwd pointing to -x, left is to -y, and bottom to -z
2. Export the craft with the following blender settings: +y forward, -z up
3. Don't export materials
