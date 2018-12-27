FROM base/archlinux:2018.06.01

RUN pacman --noconfirm -Syu \
    binutils \
    make \
    gdb \
    arm-none-eabi-gcc \
    arm-none-eabi-gdb \
    arm-none-eabi-newlib \
    stlink && \
    pacman --noconfirm -Scc # Clean the cruft
