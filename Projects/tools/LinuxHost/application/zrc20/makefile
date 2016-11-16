# Makefile for Linux Simple Console Application


#Compilation output
OBJS = out

#temporary output for lib generation
TEMP = temp

#Application Nick Name
APP := ZRC
app := zrc

#library to use
#LIBS_x86= -lpthread ../ipclib/client/rtisClientLib_x86.a
#LIBS_armBeagleBoard= -lpthread ../ipclib/client/rtisClientLib_armBeagleBoard.a
#LIBS_armBeagleBone= -lpthread ../ipclib/client/rtisClientLib_armBeagleBone.a
LIBS_x86= -lpthread -lrt -lcurl
LIBS_armBeagleBoard= -lpthread -lrt
LIBS_armBeagleBone= -lpthread -lrt

#gprof profiling. To check the profile run: ~$ gprof  ZRC_lnx_x86_client gmon.out -p
#GPROF= -pg
GPROF=
#gcov profiling

#include path
INCLUDES= \
	-I ../common \
	-I ../$(app) \
	-I ../../common \
	-I ../../ipclib/client \
	-I ../../ipclib/client/gdp \
	-I ../../ipclib/client/zrc \
	-I ../../ipclib/common \
	-I ../../services

#compilation tools
CC_armBeagleBoard = arm-none-linux-gnueabi-gcc
#CC_armBeagleBone = arm-arago-linux-gnueabi-gcc
# arm-arago-gnueabi-gcc is for default Angstrom that comes with BeagleBone
CC_armBeagleBone = arm-linux-gnueabihf-gcc
# arm-linux-gnueabihf-gcc is for Ubuntu on BeagleBone Black
# get it here: eewiki.net/display/linuxonarm/BeagleBone+Black#BeagleBoneBlack-ARMCrossCompiler:GCC
CC_x86 = gcc

#predefine
DEFINES = -DRNP_HOST -DNPI_RTI -DZRC_PROFILE -DFEATURE_ZRC20=TRUE -D__DEBUG_TIME__ -DSEND_UNCOMPRESSED
# -DTIMER_DEBUG -D__BIG_DEBUG__

#compilation Option
COMPILO_FLAGS_x86 = "-Wall  $(INCLUDES) $(DEFINES) $(GPROF) " 
COMPILO_FLAGS_armBeagleBoard = "-Wall  $(INCLUDES) $(DEFINES) -D__CCARM $(GPROF)  -mcpu=cortex-a8 -march=armv7-a"
COMPILO_FLAGS_armBeagleBone = "-Wall  $(INCLUDES) $(DEFINES) -D__CCARM $(GPROF)  -mcpu=cortex-a8 -march=armv7-a"

#list of object file to compile, WITHOUT using the precompile lib rtisClientLib_xxx.a
MAINAPP_OBJS= \
	$(OBJS)/$(app)_app_main.o \
	$(OBJS)/$(app)_app.o \
	$(OBJS)/$(app)_configuration.o \
	$(OBJS)/timer.o \
	$(OBJS)/time_printf.o \
	$(OBJS)/npi_rti.o \
	$(OBJS)/npi_ipc_client.o \
	$(OBJS)/configParser.o \
	$(OBJS)/RTI_Testapp.o\
	$(OBJS)/tiLogging.o

#by default, do not use the library.
PROJ_OBJS=$(MAINAPP_OBJS)

.PHONY: all clean lib create_output arch-all-x86 arch-all-armBeagleBoard arch-all-armBeagleBone exec_all_x86 exec_all_armBeagleBoard exec_all_armBeagleBone arch-all-x86 clean_obj clean_obj2 

all: \
	create_output \
	clean_obj \
	arch-all-x86 \
	clean_obj3 \
	arch-all-armBeagleBone

create_output:
	@echo "********************************************************" 
	@echo "Check existance of output folder" 
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

arch-all-x86: 
	@echo "********************************************************" 
	@echo "COMPILING $(APP) APPLICATION FOR x86" 
	@$(MAKE) COMPILO=$(CC_x86) COMPILO_FLAGS=$(COMPILO_FLAGS_x86) exec_all_x86

arch-all-armBeagleBoard: 
	@echo "********************************************************" 
	@echo "COMPILING $(APP) APPLICATION FOR ARM BEAGLE BOARD" 
	@$(MAKE) COMPILO=$(CC_armBeagleBoard) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBoard) exec_all_armBeagleBoard

arch-all-armBeagleBone: 
	@echo "********************************************************" 
	@echo "COMPILING $(APP) APPLICATION FOR ARM BEAGLE BONE" 
	@$(MAKE) COMPILO=$(CC_armBeagleBone) COMPILO_FLAGS=$(COMPILO_FLAGS_armBeagleBone) exec_all_armBeagleBone

exec_all_x86: $(OBJS)/$(APP)_lnx_x86_client

exec_all_armBeagleBoard: $(OBJS)/$(APP)_lnx_armBeagleBoard_client

exec_all_armBeagleBone: $(OBJS)/$(APP)_lnx_armBeagleBone_client

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

$(OBJS)/$(APP)_lnx_armBeagleBoard_client: $(PROJ_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(PROJ_OBJS) $(LIBS_armBeagleBoard)
	@echo "********************************************************" 

$(OBJS)/$(APP)_lnx_armBeagleBone_client: $(PROJ_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(PROJ_OBJS) $(LIBS_armBeagleBone)
	@echo "********************************************************" 

$(OBJS)/$(APP)_lnx_x86_client: $(PROJ_OBJS)
	@echo "Building target" $@ "..."
	@$(COMPILO) -o $@ $(PROJ_OBJS) $(LIBS_x86)
	@echo "********************************************************" 

$(OBJS)/$(app)_app_main.o: $(app)_app_main.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<
	
$(OBJS)/$(app)_app.o: $(app)_app.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<
	
$(OBJS)/$(app)_voice.o: $(app)_voice.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<
	
$(OBJS)/$(app)_configuration.o: $(app)_configuration.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<
	
#$(OBJS)/ZRC_Voice_lib.o: ../common/ZRC_Voice_lib.c
#	@echo "Compiling" $< "..."
#	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/timer.o: ../common/timer.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/time_printf.o: ../../common/time_printf.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/tiLogging.o: ../../common/tiLogging.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/npi_rti.o: ../../ipclib/client/npi_rti.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/npi_ipc_client.o: ../../ipclib/client/npi_ipc_client.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/configParser.o: ../common/configParser.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

$(OBJS)/RTI_Testapp.o: ../common/RTI_Testapp.c
	@echo "Compiling" $< "..."
	@$(COMPILO) $(COMPILO_FLAGS) -c -o $@  $<

