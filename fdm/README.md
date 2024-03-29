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

NOTE:

If you have downloaded a .zip file of this project that has no parent git repository, the
`git submodule` command won't work. Run the `download-dependencies.sh` script in the lib
directory instead!

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
$ ./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --sitl=../../fcs/build/libfcs.so --sitl_div=2 --save_telem=../../assets/aircraft/quad/data_output/quad_telem.csv --ws=9002 --sim_end=-1 --cmd=1234 --set=propulsion/engine\[1\]/pitch-angle-rad=1 --sitl_config=alt_sp_pid_p=0.035
```

### HITL example


```
$ ./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --serial=/dev/tty.usbmodem376D387232321 --hitl --save_telem=../../assets/aircraft/quad/data_output/quad_telem.csv --ws=9002 --sim_end=-1 --cmd=1234
```

### Real-time telemetry visualization


```
$ ./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --serial=/dev/tty.usbmodem3452345833311 --rt_telem --save_telem=../../assets/aircraft/quad/data_output/quad_telem.csv --ws=9002 --sim_end=-1 --cmd=1234
```

```
(venv) $ python j-viz.py configs/quad_config_telem.xml ../assets/aircraft/quad/data_output/quad_telem.csv --running 10
```