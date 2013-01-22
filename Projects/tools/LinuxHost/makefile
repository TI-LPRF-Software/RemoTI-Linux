# Makefile for Linux Simple Console Application


#Compilation output
OBJS = out

#temporary output for lib generation
TEMP = temp

#library to use
LIBS_x86= -lpthread 
LIBS_arm= -lpthread 

#include path
INCLUDES= -I application\
	-I ipclib \
	-I rtilib \
	-I services

#compilation tools
CC_arm = arm-none-linux-gnueabi-gcc
CC_x86 = gcc

#predefine
#DEFINES = -DRNP_HOST -D__BIG_DEBUG__
#DEFINES = -DRNP_HOST -DSRDY_INTERRUPT
DEFINES = -DRNP_HOST -DNPI_SPI=TRUE -DNPI_UART=TRUE -DNPI_I2C=TRUE -D__DEBUG_TIME__

#compilation Option
COMPILO_FLAGS_x86 = "-Wall  $(INCLUDES) $(DEFINES)" 
COMPILO_FLAGS_arm = "-Wall  $(INCLUDES) $(DEFINES) -mcpu=cortex-a8 -march=armv7-a"

#list of object file to compile for the server
SERVER_OBJS= \
	$(OBJS)/npi_lnx_ipc.o \
	$(OBJS)/npi_lnx_uart.o \
	$(OBJS)/npi_lnx_spi.o \
	$(OBJS)/npi_lnx_i2c.o \
	$(OBJS)/hal_gpio.o \
	$(OBJS)/hal_i2c.o \
	$(OBJS)/hal_spi.o 

CLIENT_OBJS= \
	$(OBJS)/rtis_lnx_ipc_client.o \
	$(OBJS)/SimpleConsole_lnx.o 


#by default, do not use the library.
#PROJ_OBJS=$(SERVER_OBJS)

.PHONY: all clean server lib create_output arch-all-x86 arch-all-arm exec_server_x86 exec_server_arm exec_client_x86 exec_client_arm arch-server-x86 exec_server_arm clean_obj clean_obj2 

all: \
	create_output \
	clean_obj \
	arch-all-x86 \
	clean_obj2 \
	arch-all-arm \

server: \
	create_output \
	clean_obj \
	arch-server-x86 \
	clean_obj2 \
	arch-server-arm \

client: \
	create_output \
	clean_obj \
	arch-client-x86 \
	clean_obj2 \
	arch-client-arm \

create_output:
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

arch-all-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_server_x86
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_client_x86

arch-all-arm: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM" 
	@$(MAKE) COMPILO=$(CC_arm) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_server_arm
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR ARM" 
	@$(MAKE) COMPILO=$(CC_arm) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_client_arm

arch-server-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_server_x86

arch-server-arm: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM" 
	@$(MAKE) COMPILO=$(CC_arm) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_server_arm

arch-client-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_client_x86

arch-client-arm: 
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR ARM" 
	@$(MAKE) COMPILO=$(CC_arm) COMPILO_FLAGS=$(COMPILO_FLAGS_arm) exec_client_arm

exec_server_x86: $(OBJS)/NPI_lnx_x86_server

exec_server_arm: $(OBJS)/NPI_lnx_arm_server

exec_client_x86: $(OBJS)/NPI_lnx_x86_client

exec_client_arm: $(OBJS)/NPI_lnx_arm_client


arch-sbl-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SERIAL BOOT LOADER APPLICATION FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_sbl_x86

exec_sbl_x86: $(OBJS)/RTI_lnx_x86_client_sbl

clean_obj:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean_obj2:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean:
	@echo "********************************************************" 
	@echo "Cleaning all" 
	-if test -d $(OBJS); then rm -rf $(OBJS); fi

$(OBJS)/NPI_lnx_arm_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_arm)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_x86_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_x86)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_arm_client: $(CLIENT_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(CLIENT_OBJS) $(LIBS_arm)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_x86_client: $(CLIENT_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(CLIENT_OBJS) $(LIBS_x86)
	@echo "********************************************************" 

$(OBJS)/npi_lnx_ipc.o: ipclib/npi_lnx_ipc.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/npi_lnx_spi.o: ipclib/npi_lnx_spi.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/npi_lnx_uart.o: ipclib/npi_lnx_uart.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/npi_lnx_i2c.o: ipclib/npi_lnx_i2c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_spi.o: ipclib/hal_spi.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_i2c.o: ipclib/hal_i2c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_gpio.o: ipclib/hal_gpio.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/rtis_lnx_ipc_client.o: rtilib/rtis_lnx_ipc_client.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/SimpleConsole_lnx.o: application/SimpleConsole_lnx.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	


