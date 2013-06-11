# Makefile for Linux Simple Console Application


#Compilation output
OBJS = out

#temporary output for lib generation
TEMP = temp

#Application Nick Name
APP := SIMPLE
app := simple

#library to use
#LIBS_x86= -lpthread ../ipclib/client/rtisClientLib_x86.a
#LIBS_armBeagleBoard= -lpthread ../ipclib/client/rtisClientLib_armBeagleBoard.a
#LIBS_armBeagleBone= -lpthread ../ipclib/client/rtisClientLib_armBeagleBone.a
LIBS_x86= -lpthread 
LIBS_armBeagleBoard= -lpthread 
LIBS_armBeagleBone= -lpthread 

#gprof profiling. To check the profile run (e.g): ~$ gprof RTI_lnx_x86_client gmon.out -p
#GPROF= -pg
GPROF=
#gcov profiling

#include path
INCLUDES= \
	-I ../common \
	-I ../$(app) \
	-I ../../ipclib/client \
	-I ../../ipclib/common \
	-I ../../services

#compilation tools
CC_armBeagleBoard = arm-none-linux-gnueabi-gcc
CC_armBeagleBone = arm-arago-linux-gnueabi-gcc
CC_x86 = gcc

#compilation Option
COMPILO_FLAGS_x86 = "-Wall  $(INCLUDES) $(DEFINES) $(GPROF) " 
COMPILO_FLAGS_armBeagleBoard = "-Wall  $(INCLUDES) $(DEFINES) $(GPROF)  -mcpu=cortex-a8 -march=armv7-a"
COMPILO_FLAGS_armBeagleBone = "-Wall  $(INCLUDES) $(DEFINES) $(GPROF)  -mcpu=cortex-a8 -march=armv7-a"

#predefine
#DEFINES = -DRNP_HOST -D__BIG_DEBUG__
#DEFINES = -DRNP_HOST -DSRDY_INTERRUPT
DEFINES = -DRNP_HOST -DNPI_RTI

#list of object file to compile, WITHOUT using the precompile lib rtisClientLib_xxx.a
MAINAPP_OBJS_x86= \
	$(OBJS)/$(app)_app_main.o \
	$(OBJS)/$(app)_app.o \
	$(OBJS)/timer.o \
	$(OBJS)/npi_rti.o \
	$(OBJS)/npi_ipc_client.o 

MAINAPP_OBJS_armBeagleBoard= \
	$(OBJS)/$(app)_app_main_armBeagleBoard.o \
	$(OBJS)/$(app)_app_armBeagleBoard.o \
	$(OBJS)/timer_armBeagleBoard.o \
	$(OBJS)/npi_rti_armBeagleBoard.o \
	$(OBJS)/npi_ipc_client_armBeagleBoard.o 

MAINAPP_OBJS_armBeagleBone= \
	$(OBJS)/$(app)_app_main_armBeagleBone.o \
	$(OBJS)/$(app)_app_armBeagleBone.o \
	$(OBJS)/timer_armBeagleBone.o \
	$(OBJS)/npi_rti_armBeagleBone.o \
	$(OBJS)/npi_ipc_client_armBeagleBone.o 

#by default, do not use the library.
PROJ_OBJS_x86=$(MAINAPP_OBJS_x86) 
PROJ_OBJS_armBeagleBoard=$(MAINAPP_OBJS_armBeagleBoard)
PROJ_OBJS_armBeagleBone=$(MAINAPP_OBJS_armBeagleBone)

.PHONY: all clean distclean mrproper create_output arch-all-x86 arch-all-armBeagleBoard arch-all-armBeagleBone exec_main_x86 exec_main_armBeagleBoard exec_main_armBeagleBone distclean mrproper 

all: \
	create_output \
	arch-all-x86 \
	arch-all-armBeagleBoard \
	arch-all-armBeagleBone \

create_output:
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

arch-all-x86: 
	@echo "********************************************************" 
	@echo "COMPILING $(APP) APPLICATION FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_main_x86

arch-all-armBeagleBoard: 
	@echo "********************************************************" 
	@echo "COMPILING $(APP) APPLICATION FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_main_armBeagleBoard

arch-all-armBeagleBone: 
	@echo "********************************************************" 
	@echo "COMPILING $(APP) APPLICATION FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_main_armBeagleBone

exec_main_x86: $(OBJS)/$(APP)_lnx_x86_client

exec_main_armBeagleBoard: $(OBJS)/$(APP)_lnx_armBeagleBoard_client

exec_main_armBeagleBone: $(OBJS)/$(APP)_lnx_armBeagleBone_client


clean distclean:
	@echo "********************************************************" 
	@echo "Cleaning Existing object" 
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi

mrproper:
	@echo "********************************************************" 
	@echo "Cleaning all" 
	-if test -d $(OBJS); then rm -rf $(OBJS); fi

$(OBJS)/$(APP)_lnx_armBeagleBoard_client: $(PROJ_OBJS_armBeagleBoard)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(PROJ_OBJS_armBeagleBoard) $(LIBS_armBeagleBoard)
	@echo "********************************************************" 

$(OBJS)/$(APP)_lnx_armBeagleBone_client: $(PROJ_OBJS_armBeagleBone)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(PROJ_OBJS_armBeagleBone) $(LIBS_armBeagleBone)
	@echo "********************************************************" 

$(OBJS)/$(APP)_lnx_x86_client: $(PROJ_OBJS_x86)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(PROJ_OBJS_x86) $(LIBS_x86)
	@echo "********************************************************" 

$(OBJS)/$(app)_app_main.o $(OBJS)/$(app)_app_main_armBeagleBoard.o $(OBJS)/$(app)_app_main_armBeagleBone.o: $(app)_app_main.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<
$(OBJS)/$(app)_app.o $(OBJS)/$(app)_app_armBeagleBoard.o $(OBJS)/$(app)_app_armBeagleBone.o: $(app)_app.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/timer.o $(OBJS)/timer_armBeagleBoard.o $(OBJS)/timer_armBeagleBone.o: ../common/timer.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/npi_rti.o $(OBJS)/npi_rti_armBeagleBoard.o $(OBJS)/npi_rti_armBeagleBone.o: ../../ipclib/client/npi_rti.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/npi_ipc_client.o $(OBJS)/npi_ipc_client_armBeagleBoard.o $(OBJS)/npi_ipc_client_armBeagleBone.o: ../../ipclib/client/npi_ipc_client.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

