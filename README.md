# dt-duckiebot-interface

Status:
[![Build Status](http://build-arm.duckietown.org/job/Docker%20Autobuild%20-%20dt-duckiebot-interface/badge/icon.svg)](http://build-arm.duckietown.org/job/Docker%20Autobuild%20-%20dt-duckiebot-interface/)
[![Docker Hub](https://img.shields.io/docker/pulls/duckietown/dt-duckiebot-interface.svg)](https://hub.docker.com/r/duckietown/dt-duckiebot-interface)

Repository containing all the necessary drivers to start sensors and actuators.
It should not contain any high-level functionality.


## How to launch manually

```$ docker -H <Hostname>.local run --name duckiebot-interface -v /data:/data --privileged --network=host -dit --restart unless-stopped -e ROBOT_TYPE=<ROBOT_TYPE> duckietown/duckiebot-interface:ente-arm64v8```

By default, `ROBOT_TYPE` is duckiebot, and you can set it to watchtower or traffic_light if you use them.

## Jetson Orin Nano build

The Jetson Orin Nano requires newer versions of several camera libraries to work. Since the same libraries are also used by the Jetson Nano and they are not backwards compatible there is in this repo a separate `Dockerfile.orin` to build the container for the Orin Nano. The command to do this is:

```
dts devel build -H jetson --file Dockerfile.orin --tag ente-arm64v8-orin --pull --no-cache
```

To push the image you can optionally add the `--push` flag.

### Note

In order to ensure compatibility this image should be built on a Jetson Orin Nano.


## Development
There is a bug with numpy, it requires passing the variable `OPENBLAS_NUM_THREADS=1` to the container.

In order to attach VSCode to the running container we need to add the flag `--security-opt seccomp=unconfined` to the container (see why [here](https://askubuntu.com/questions/1405417/20-04-vs-22-04-inside-docker-with-a-16-04-host-thread-start-failures)).

we need to mount the volumes as `RW` with the `-RW` flag in order to edit files inside the devcontainer.

### Virtual Robots
The virtual robots require in addition to the standard flags also the `-e OPENBLAS_NUM_THREADS=1` and `--security-opt seccomp=unconfined` flags in order to work correctly on `amd64` machines.

```bash
dts devel run -H VIRTUAL_ROBOT -RW -c bash -- -e OPENBLAS_NUM_THREADS=1 --security-opt seccomp=unconfined -v /data/ramdisk/dtps:/dtps -e DT_SUPERUSER=1
```

### Real Robots
```bash
dts devel run -H REAL_ROBOT -RW -c bash --detach -- -v /data/ramdisk/dtps:/dtps -e DT_SUPERUSER=1 --privileged
```
