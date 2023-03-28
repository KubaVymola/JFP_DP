# Dependencies

* C++11
* CMake

### MacOS

* asio (needed instead of boost if compiling websocketpp from source with ASIO_STANDALONE in main.cpp)
    * brew install asio
* boost (needed instead of asio if using websocketpp package)
    * brew install boost
* websocketpp
    * brew install websocketpp

### Ubuntu


* apt install build-essential cmake

* asio
    * apt install libasio-dev
* websocket
    * apt install libwebsocketpp-dev

# Building

```
$ git submodule update --init --recursive
$ mkdir build && cd build
$ cmake ..
$ cmake --build . -j4
```

# Running

```
$ ./fdm <script_file> [<output_file> [<output_file> ...]] <options>
```

### SITL example

```
$ ./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --sitl=../../fcs/build/libfcs.dylib --sitl_div=5 --ws=9002 --set=propulsion/engine\[1\]/pitch-angle-rad=1 --sitl_config=alt_sp_pid_p=0.035 --sim_end=-1
```

### HITL example

```
$ ./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --serial=/dev/tty.usbmodem1747313034341 --hitl --save_telem=../../assets/aircraft/quad/data_output/quad_telem.csv --ws=9002 --sim_end=-1
```

### Real-time telemetry visualization

```
./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --serial=/dev/tty.usbmodem1747313034341 --rt_telem --save_telem=../../assets/aircraft/quad/data_output/quad_telem.csv --ws=9002 --sim_end=-1
```

```
python j-viz.py quad_config_telem.xml ../../assets/aircraft/quad/data_output/quad_telem.csv --running 10
```