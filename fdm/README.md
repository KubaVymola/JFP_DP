# Dependencies

* C++11
* CMake

## MacOS

* asio (needed instead of boost if compiling websocketpp from source with ASIO_STANDALONE in main.cpp)
    * brew install asio
* boost (needed instead of asio if using websocketpp package)
    * brew install boost
* websocketpp
    * brew install websocketpp

## Ubuntu

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
e.g.
$ ./fdm script/quad_script_001.xml aircraft/quad/output_def/quad_out_def.xml --root_dir=../../assets --sitl=../../fcs/build/libfcs.dylib --ws=9002 --set=propulsion/engine\[1\]/pitch-angle-rad=1 --sim_end=-1
```
