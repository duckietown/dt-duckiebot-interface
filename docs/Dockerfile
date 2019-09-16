FROM duckietown/dt-duckiebot-interface:daffy

RUN apt-get update; apt-get install -y git

RUN python -m pip install sphinx==1.7.0 sphinx-rtd-theme sphinx-autobuild mock

RUN cd /; git clone https://github.com/AleksandarPetrov/napoleon; cd /napoleon; python setup.py install -f

CMD "/ros_entrypoint.sh"
