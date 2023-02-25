# Dependencies

- gcc-arm-none-eabi

# Building

### Make independent firmware to control a vehicle

```
$ make [clean] indep [flash] TARGET=<target>
```

### Make HITL firmware to control simulated vehicle from MCU via serial interface

```
$ make [clean] hitl [flash] TARGET=<target> [AUTOARM=1]
```

### Make SITL shared library to control simulated vehicle via software interface

```
$ make [clean] sitl
```

# Supported TARGETs

- STM32F405

### Reset MCU with OpenOCD

```
$ openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; reset; exit"
```