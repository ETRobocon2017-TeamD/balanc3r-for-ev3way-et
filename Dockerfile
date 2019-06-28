FROM ev3dev/ev3dev-jessie-ev3-generic

# get python-config - https://packages.debian.org/jessie/python3-dev
# tar xf gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.xz - https://packages.debian.org/jessie/xz-utils
RUN apt-get update && \
    apt-get install --yes python3-dev:armel xz-utils

# install the cross-compiler toolchain
ADD https://releases.linaro.org/archive/14.04/components/toolchain/binaries/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.xz /opt/
RUN cd /opt && \
    tar xf gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.xz && \
    rm gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux.tar.xz
COPY docker-cross/ev3dev-jessie/toolchain.cmake.armel /opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/toolchain.cmake
