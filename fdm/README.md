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
$ ./fdm <path_to_assets_root> <relative_path_from_assets_root_to_script>
e.g.
$ ./fdm ../../assets/ script/rocket_script_001.xml
```
