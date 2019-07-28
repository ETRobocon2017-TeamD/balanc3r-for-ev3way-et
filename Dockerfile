# FROM ev3dev/debian-jessie-armel-cross
FROM ev3dev/debian-stretch-armel-cross

# get python-config - https://packages.debian.org/jessie/python3-dev
RUN sudo apt-get update && \
    sudo apt-get install --yes python3-dev:armel
