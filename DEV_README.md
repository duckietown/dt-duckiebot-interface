There is a bug with numpy, it requires passing the variable `OPENBLAS_NUM_THREADS=1` to the container.

In order to attach VSCode to the running container we need to add the flag `--security-opt seccomp=unconfined` to the container (see why [here](https://askubuntu.com/questions/1405417/20-04-vs-22-04-inside-docker-with-a-16-04-host-thread-start-failures)).

we need to mount the volumes as `RW` with the `-RW` flag in order to edit files inside the devcontainer.

```bash
dts devel run -H virtualtuxdrone -RW -c bash -- -e OPENBLAS_NUM_THREADS=1 --security-opt seccomp=unconfined -v dtps:/dtps
```
