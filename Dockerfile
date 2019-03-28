FROM duckietown/rpi-ros-kinetic-base:master18

RUN [ "cross-build-start" ]

COPY requirements.txt /requirements.txt

ENV READTHEDOCS True
RUN pip install -r /requirements.txt

RUN mkdir /home/duckiebot-interface/
COPY . /home/duckiebot-interface

ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
RUN /bin/bash -c "cd /home/duckiebot-interface/ && source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"

RUN echo "source /home/duckiebot-interface/docker_setup.sh" >> ~/.bashrc
RUN bash -c "source /home/duckiebot-interface/docker_setup.sh"

RUN [ "cross-build-end" ]

WORKDIR /home/duckiebot-interface

CMD [ "./run_all_drivers.sh" ]
