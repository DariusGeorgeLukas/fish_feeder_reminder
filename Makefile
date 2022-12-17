#System-dependent configuration
#####################################################################################
#Configure your msp430-gcc path
MSP430_GCC := $(abspath $(dir 	$(HOME)/ti/msp430-gcc/))
#####################################################################################

#Configure Device
DEVICE = MSP430FR2433

MICRO_CAC_DIR = $(abspath $(dir $(HOME)/projects/gitHub/micro_CAC/))

-include $(MICRO_CAC_DIR)/Makefile

