FROM ubuntu:18.04

RUN ln -fs /usr/share/zoneinfo/Europe/Zurich /etc/localtime

RUN apt-get update; apt-get install -y python3-sphinx python3-pip git make

RUN pip3 install --no-cache sphinx sphinx-rtd-theme sphinx-autobuild mock

RUN cd /; git clone https://github.com/AleksandarPetrov/napoleon; cd /napoleon; python3 setup.py install -f

RUN cd /; git clone https://github.com/AleksandarPetrov/dt-ros-commons; cd dt-ros-commons; git checkout devel20;

CMD "/ros_entrypoint.sh"

