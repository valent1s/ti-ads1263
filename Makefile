obj-m += ti-ads1263.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: ads1263-overlay.dts
	dtc -@ -I dts -O dtb -o ads1263-overlay.dtbo ads1263-overlay.dts
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf ads1263-overlay.dtbo