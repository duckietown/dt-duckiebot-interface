# How to build the docs

Building the docs should not be too tricky if you follow the steps outlined here.

First, build the container with the actual code. So go in the parent directory (the repository directory) and run `dts devel build`. Keep in mind that you should build it on your computer, not a duckiebot. Take a note of the resulting image name.

Check if the `FROM` command in the `Dockerfile` in the `docs/` folder corresponds to the image that you just build. If yes, you can build the docs image by running:  

    docker build -t sphinx-docs docs/
   
Once the image is ready, you can finally build the docs by using it:

    docker run -it --rm -v $(pwd):/code/catkin_ws/src/dt-car-interface/ sphinx-docs bash -c "cd /code/catkin_ws/src/dt-car-interface/docs; rm -r build; make html"

    
Note that for these commands to run correctly you need to run them from the parent folder (the repository folder), not from within `docs\`.

The resulting `html` code should be now available under `docs/build`.