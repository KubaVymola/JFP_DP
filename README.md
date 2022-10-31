# Dependencies

* C++11

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