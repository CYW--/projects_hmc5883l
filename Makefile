PWD=$(shell pwd)

obj-m = sensor_h43.o

all:
	make ARCH=${ARCH} CROSS_COMPILE=${CC} -C ${LINUX_SRC} M=${PWD} modules
