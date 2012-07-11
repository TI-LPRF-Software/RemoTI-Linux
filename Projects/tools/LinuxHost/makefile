# Makefile for Linux Simple Console Application
#
# $Revision: 20578 $
# $Date: 2009-08-17 14:27:28 -0700 (Mon, 17 Aug 2009) $

OBJS = debug

LIBS= -lpthread
CC = arm-none-linux-gnueabi-gcc

#DEFINES = -DRNP_HOST -D__BIG_DEBUG__
#DEFINES = -DRNP_HOST -DSRDY_INTERRUPT
DEFINES = -DRNP_HOST
DEFINES_ALL = -DHAL_SPI=TRUE -DHAL_UART=TRUE -DHAL_I2C=TRUE
DEFINES_SPI = -DHAL_SPI=TRUE -DHAL_UART=FALSE -DHAL_I2C=FALSE
DEFINES_UART = -DHAL_SPI=FALSE -DHAL_UART=TRUE -DHAL_I2C=FALSE
DEFINES_I2C = -DHAL_SPI=FALSE -DHAL_UART=FALSE -DHAL_I2C=TRUE

INCLUDES= -I application\
	-I ipclib \
	-I rtilib \
	-I services
           
MAINAPP_OBJS= \
	$(OBJS)/SimpleConsole_lnx.o
	
RTILIB_OBJS= \
	$(OBJS)/rtis_lnx.o

NPISPILIB_OBJS= \
	$(OBJS)/npi_lnx_spi.o

NPIUARTLIB_OBJS= \
	$(OBJS)/npi_lnx_uart.o

NPII2CLIB_OBJS= \
	$(OBJS)/npi_lnx_i2c.o

HAL_LIB_OBJS= \
	$(OBJS)/hal_gpio.o

HAL_I2C_LIB_OBJS= \
	$(OBJS)/hal_i2c.o

HAL_SPI_LIB_OBJS= \
	$(OBJS)/hal_spi.o 


PROJ_ALL_OBJS= $(MAINAPP_OBJS) $(RTILIB_OBJS) $(NPIUARTLIB_OBJS) $(NPISPILIB_OBJS) $(HAL_SPI_LIB_OBJS) $(NPII2CLIB_OBJS) $(HAL_I2C_LIB_OBJS) $(HAL_LIB_OBJS)
PROJ_UART_OBJS= $(MAINAPP_OBJS) $(RTILIB_OBJS) $(NPIUARTLIB_OBJS) $(HAL_LIB_OBJS) 
PROJ_SPI_OBJS= $(MAINAPP_OBJS) $(RTILIB_OBJS) $(NPISPILIB_OBJS) $(HAL_SPI_LIB_OBJS) $(HAL_LIB_OBJS)
PROJ_I2C_OBJS= $(MAINAPP_OBJS) $(RTILIB_OBJS) $(NPII2CLIB_OBJS) $(HAL_I2C_LIB_OBJS) $(HAL_LIB_OBJS)


all: CCFLAGS = -Wall  $(INCLUDES) $(DEFINES) $(DEFINES_ALL) -mcpu=cortex-a8 -march=armv7-a 
all: erase_obj $(OBJS)/SimpleConsole
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

clean:
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi
	-if test -f $(OBJS)/SimpleConsole; then rm -f $(OBJS)/SimpleConsole; fi
	-if test -f $(OBJS)/SimpleConsole_uart; then rm -f $(OBJS)/SimpleConsole_uart; fi
	-if test -f $(OBJS)/SimpleConsole_spi; then rm -f $(OBJS)/SimpleConsole_spi; fi
	-if test -f $(OBJS)/SimpleConsole_i2c; then rm -f $(OBJS)/SimpleConsole_i2c; fi

uart: CCFLAGS = -Wall  $(INCLUDES) $(DEFINES) $(DEFINES_UART) -mcpu=cortex-a8 -march=armv7-a 
uart: erase_obj $(OBJS)/SimpleConsole_uart

spi: CCFLAGS = -Wall  $(INCLUDES) $(DEFINES) $(DEFINES_SPI) -mcpu=cortex-a8 -march=armv7-a 
spi: erase_obj $(OBJS)/SimpleConsole_spi

i2c: CCFLAGS = -Wall  $(INCLUDES) $(DEFINES) $(DEFINES_I2C) -mcpu=cortex-a8 -march=armv7-a 
i2c: erase_obj $(OBJS)/SimpleConsole_i2c

erase_obj:
	-if test -d $(OBJS); then rm -f $(OBJS)/*.o; fi
	-if test ! -d $(OBJS); then mkdir $(OBJS); fi

$(OBJS)/SimpleConsole: $(PROJ_ALL_OBJS)
	$(CC) -o $@ $(PROJ_ALL_OBJS) $(LIBS) $(CCFLAGS)
	
$(OBJS)/SimpleConsole_uart: $(PROJ_UART_OBJS)
	$(CC) -o $@ $(PROJ_UART_OBJS) $(LIBS) $(CCFLAGS)

$(OBJS)/SimpleConsole_spi: $(PROJ_SPI_OBJS)
	$(CC) -o $@ $(PROJ_SPI_OBJS) $(LIBS) $(CCFLAGS)

$(OBJS)/SimpleConsole_i2c: $(PROJ_I2C_OBJS)
	$(CC) -o $@ $(PROJ_I2C_OBJS) $(LIBS) $(CCFLAGS)

$(OBJS)/SimpleConsole_lnx.o: application/SimpleConsole_lnx.c
	$(CC) -c -o $@ $(CCFLAGS) $<
	
$(OBJS)/rtis_lnx.o: rtilib/rtis_lnx.c
	$(CC) -c -o $@ $(CCFLAGS) $<

$(OBJS)/npi_lnx_spi.o: ipclib/npi_lnx_spi.c
	$(CC) -c -o $@ $(CCFLAGS) $<

$(OBJS)/npi_lnx_uart.o: ipclib/npi_lnx_uart.c
	$(CC) -c -o $@ $(CCFLAGS) $<

$(OBJS)/npi_lnx_i2c.o: ipclib/npi_lnx_i2c.c
	$(CC) -c -o $@ $(CCFLAGS) $<

$(OBJS)/hal_spi.o: ipclib/hal_spi.c
	$(CC) -c -o $@ $(CCFLAGS) $<

$(OBJS)/hal_i2c.o: ipclib/hal_i2c.c
	$(CC) -c -o $@ $(CCFLAGS) $<

$(OBJS)/hal_gpio.o: ipclib/hal_gpio.c
	$(CC) -c -o $@ $(CCFLAGS) $<

.PHONY: all clean uart spi i2c
