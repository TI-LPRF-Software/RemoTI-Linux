# Makefile for Linux Simple Console Application


#Compilation output
OBJS = out

#temporary output for lib generation
TEMP = temp

#library to use
LIBS_x86= -lpthread
# -lusb-1.0
LIBS_armBeagleBoard= -lpthread
# -lusb-1.0
LIBS_armBeagleBone= -lpthread
# -lusb-1.0

#include path
INCLUDES= -I application\
	-I ipclib \
	-I rtilib \
	-I services

#compilation tools
CC_armBeagleBoard = arm-none-linux-gnueabi-gcc
CC_armBeagleBone = arm-arago-linux-gnueabi-gcc
CC_x86 = gcc

#predefine
#DEFINES = -DRNP_HOST -D__BIG_DEBUG__
#DEFINES = -DRNP_HOST -DSRDY_INTERRUPT -DNPI_UNIX
DEFINES = -DRNP_HOST -DNPI_SPI=TRUE -DNPI_UART=TRUE -DNPI_I2C=TRUE -D__DEBUG_TIME__ -DSRDY_INTERRUPT -D__DEBUG_TIME__I2C
# -D__DEBUG_TIME__HID -D__DEBUG_MUTEX__ -DNPI_HID=TRUE 

#compilation Option
COMPILO_FLAGS_x86 = "-Wall  $(INCLUDES) $(DEFINES)" 
COMPILO_FLAGS_armBeagleBoard = "-Wall  $(INCLUDES) $(DEFINES) -mcpu=cortex-a8 -march=armv7-a"
COMPILO_FLAGS_armBeagleBone = "-Wall  $(INCLUDES) $(DEFINES) -mcpu=cortex-a8 -march=armv7-a"

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
	arch-all-armBeagleBoard \
	clean_obj3 \
	arch-all-armBeagleBone \

server: \
	create_output \
	clean_obj \
	arch-server-x86 \
	clean_obj2 \
	arch-server-armBeagleBoard \
	clean_obj3 \
	arch-server-armBeagleBone \

client: \
	create_output \
	clean_obj \
	arch-client-x86 \
	clean_obj2 \
	arch-client-armBeagleBoard \
	clean_obj3 \
	arch-client-armBeagleBone \

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

arch-all-armBeagleBoard: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_server_armBeagleBoard
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_client_armBeagleBoard

arch-all-armBeagleBone: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_server_armBeagleBone
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_client_armBeagleBone

arch-server-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_server_x86

arch-server-armBeagleBoard: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_server_armBeagleBoard

arch-server-arm: 
	@echo "********************************************************" 
	@echo "COMPILING SERVER FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_server_armBeagleBoard

arch-client-x86: 
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_client_x86

arch-client-arm: 
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_client_armBeagleBoard

arch-client-armBeagleBone: 
	@echo "********************************************************" 
	@echo "COMPILING SIMPLE APP FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_client_armBeagleBone

exec_server_x86: $(OBJS)/NPI_lnx_x86_server

exec_server_armBeagleBoard: $(OBJS)/NPI_lnx_armBeagleBoard_server

exec_server_armBeagleBone: $(OBJS)/NPI_lnx_armBeagleBone_server

exec_client_x86: $(OBJS)/NPI_lnx_x86_client

exec_client_armBeagleBoard: $(OBJS)/NPI_lnx_armBeagleBoard_client

exec_client_armBeagleBone: $(OBJS)/NPI_lnx_armBeagleBone_client


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

clean_obj3:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

clean:
	@echo "********************************************************" 
	@echo "Cleaning all" 
	-if test -d $(OBJS); then rm -rf $(OBJS); fi

$(OBJS)/NPI_lnx_armBeagleBoard_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_armBeagleBoard)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_armBeagleBone_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_armBeagleBone)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_x86_server: $(SERVER_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(SERVER_OBJS) $(LIBS_x86)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_armBeagleBoard_client: $(CLIENT_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(CLIENT_OBJS) $(LIBS_armBeagleBoard)
	@echo "********************************************************" 

$(OBJS)/NPI_lnx_armBeagleBone_client: $(CLIENT_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(CLIENT_OBJS) $(LIBS_armBeagleBone)
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

#$(OBJS)/npi_lnx_hid.o: ipclib/npi_lnx_hid.c
#	@echo "Compiling" $< "..."
#	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_spi.o: ipclib/hal_spi.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_i2c.o: ipclib/hal_i2c.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

#$(OBJS)/hid.o: ipclib/hid.c
#	@echo "Compiling" $< "..."
#	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/hal_gpio.o: ipclib/hal_gpio.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/rtis_lnx_ipc_client.o: rtilib/rtis_lnx_ipc_client.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<

$(OBJS)/SimpleConsole_lnx.o: application/SimpleConsole_lnx.c
	@echo "Compiling" $< "..."
	@$(COMPILO) -c -o $@ $(COMPILO_FLAGS) $<
	


