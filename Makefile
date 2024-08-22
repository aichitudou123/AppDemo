KERNELDIR := /home/xjh/Linux/MX6ULL
CURRENT_PATH := $(shell pwd)

obj-m := irq.o imx6uirq.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
