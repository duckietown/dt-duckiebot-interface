#FROM ubuntu:18.04
#
#RUN ln -fs /usr/share/zoneinfo/Europe/Zurich /etc/localtime
#
#RUN apt-get update; apt-get install -y python-pip git make
#
#RUN python -m pip install --no-cache sphinx sphinx-rtd-theme sphinx-autobuild mock
#
#RUN cd /; git clone https://github.com/AleksandarPetrov/napoleon; cd /napoleon; python setup.py install -f
#
#RUN cd /; git clone https://github.com/AleksandarPetrov/dt-ros-commons; cd dt-ros-commons; git checkout devel20;
#
#CMD "/ros_entrypoint.sh"

FROM duckietown/duckiebot-interface:devel20

RUN apt-get update; apt-get install -y git

RUN python -m pip install sphinx==1.7.0 sphinx-rtd-theme sphinx-autobuild mock

RUN cd /; git clone https://github.com/AleksandarPetrov/napoleon; cd /napoleon; python setup.py install -f

CMD "/ros_entrypoint.sh"