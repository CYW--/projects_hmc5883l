PWD=$(shell pwd)

obj-m := h43.o
h43-objs = sensor_h43.o
obj-m := h43_client.o
obj-m := h43_driver.o

all:
	make ARCH=${ARCH} CROSS_COMPILE=${CC} -C ${LINUX_SRC} M=${PWD} modules

clean:
	@rm -rf *.ko
	@rm -rf *.o
	@rm -rf *.mod.c
	@rm -rf *.symvers
	@rm -rf *.order
	@rm -rf .*.cmd
	@rm -rf .tmp_versions/
