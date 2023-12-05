###########################################################################
## Makefile generated for component 'hardwareModel_5'. 
## 
## Makefile     : hardwareModel_5.mk
## Generated on : Mon Nov 27 14:38:29 2023
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/hardwareModel_5.elf
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile

PRODUCT_NAME              = hardwareModel_5
MAKEFILE                  = hardwareModel_5.mk
MATLAB_ROOT               = /usr/local/MATLAB/R2023b
MATLAB_BIN                = /usr/local/MATLAB/R2023b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /home/yonathan/Documentos/Control/Final
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = None
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..
SLIB_PATH                 = /home/root/Documents/MATLAB/R2023b/ArduinoStaticLibrary/ArduinoNano33IoT/FasterRuns
SSLIB_PATH                = /home/root/Documents/MATLAB/R2023b/ArduinoStaticLibrary/ArduinoNano33IoT/FasterRuns
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Arduino ARM
# Supported Version(s):    
# ToolchainInfo Version:   2023b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# ARDUINO_ROOT
# ARDUINO_PACKAGES_TOOLS_ROOT
# ARDUINO_PORT
# ARDUINO_MCU
# ARDUINO_BAUD
# ARDUINO_PROTOCOL
# ARDUINO_F_CPU

#-----------
# MACROS
#-----------

PRODUCT_HEX                 = $(RELATIVE_PATH_TO_ANCHOR)/$(PRODUCT_NAME).hex
PRODUCT_BIN                 = $(RELATIVE_PATH_TO_ANCHOR)/$(PRODUCT_NAME).bin
ARDUINO_TOOLS               = $(ARDUINO_PACKAGES_TOOLS_ROOT)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -Wl,--end-group -lm -gcc -lcomm -lcore

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: Arduino ARM Assembler
AS_PATH = $(ARDUINO_TOOLS)
AS = "$(AS_PATH)/arm-none-eabi-gcc"

# C Compiler: Arduino ARM C Compiler
CC_PATH = $(ARDUINO_TOOLS)
CC = "$(CC_PATH)/arm-none-eabi-gcc"

# Linker: Arduino ARM Linker
LD_PATH = $(ARDUINO_TOOLS)
LD = "$(LD_PATH)/arm-none-eabi-gcc"

# C++ Compiler: Arduino ARM C++ Compiler
CPP_PATH = $(ARDUINO_TOOLS)
CPP = "$(CPP_PATH)/arm-none-eabi-g++"

# C++ Linker: Arduino ARM C++ Linker
CPP_LD_PATH = $(ARDUINO_TOOLS)
CPP_LD = "$(CPP_LD_PATH)/arm-none-eabi-gcc"

# Archiver: Arduino ARM Archiver
AR_PATH = $(ARDUINO_TOOLS)
AR = "$(AR_PATH)/arm-none-eabi-ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Binary Converter: Binary Converter
OBJCOPY_PATH = $(ARDUINO_PACKAGES_TOOLS_ROOT)/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin
OBJCOPY = "$(OBJCOPY_PATH)/arm-none-eabi-objcopy"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =



#---------------------------
# Model-Specific Options
#---------------------------

ASFLAGS = -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  -Wall -x assembler-with-cpp $(ASFLAGS_ADDITIONAL) $(DEFINES) $(INCLUDES) -c

CFLAGS = -std=gnu11 -Os -c -w -ffunction-sections -fdata-sections  -nostdlib  --param max-inline-insns-single=500  -Dprintf=iprintf  -DARDUINO=10801  -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@" -g

LDFLAGS = -Os -Wl,-Map="$(PRODUCT_NAME).map" -Wl,--gc-sections -g

SHAREDLIB_LDFLAGS = -g

CPPFLAGS = -std=gnu++11 -fno-threadsafe-statics -fno-rtti -fno-exceptions  -Os -c -w -ffunction-sections -fdata-sections  -nostdlib  --param max-inline-insns-single=500  -Dprintf=iprintf  -DARDUINO=10801  -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@" -g

CPP_LDFLAGS = -Os -Wl,-Map="$(PRODUCT_NAME).map" -Wl,--gc-sections -g

CPP_SHAREDLIB_LDFLAGS = -g

ARFLAGS = ruvs

OBJCOPYFLAGS_BIN = -O binary $(PRODUCT) $(PRODUCT_BIN)

DOWNLOAD_FLAGS = 

EXECUTE_FLAGS = 

MAKE_FLAGS = -f $(MAKEFILE)

###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/hardwareModel_5.elf
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(MATLAB_ROOT)/toolbox/target/shared/svd/common/include -I/root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/include -I$(START_DIR)/hardwareModel_5_ert_rtw -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/include -I$(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src -I$(MATLAB_ROOT)/toolbox/coder/rtiostream/src -I$(ARDUINO_SAMD_ROOT)/tools/CMSIS/$(SAMD_CMSIS_VERSION)/CMSIS/Include -I$(ARDUINO_SAMD_ROOT)/tools/CMSIS-Atmel/$(SAMD_CMSIS_ATMEL_VERSION)/CMSIS/Device/ATMEL -I$(ARDUINO_SAMD_ROOT)/hardware/samd/$(SAMD_LIB_VERSION)/cores/arduino -I$(ARDUINO_SAMD_ROOT)/hardware/samd/$(SAMD_LIB_VERSION)/variants/nano_33_iot -I/root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/include -I/root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/scheduler/include -I$(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/include -I$(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/xcp/include -I/root/Documents/MATLAB/SupportPackages/R2023b/3P.instrset/arduinoide.instrset/arduino-1.8.19/portable/packages/arduino/hardware/samd/1.8.9/libraries/Wire -I/root/Documents/MATLAB/SupportPackages/R2023b/3P.instrset/arduinoide.instrset/arduino-1.8.19/portable/packages/arduino/hardware/samd/1.8.9/libraries/SPI -I/root/Documents/MATLAB/SupportPackages/R2023b/3P.instrset/arduinoide.instrset/arduino-1.8.19/libraries/Servo/src -I/root/Documents/MATLAB/SupportPackages/R2023b/3P.instrset/arduinoide.instrset/arduino-1.8.19/libraries/Servo/src/samd

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -DXCP_ADDRESS_GRANULARITY=XCP_ADDRESS_GRANULARITY_BYTE -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_TIMERID=10 -DMW_PRESCALAR=128 -DMW_TIMERCOUNT=625 -DMW_SCHEDULERCOUNTER=1 -DARDUINO_NUM_SERIAL_PORTS=2 -D_RTT_BAUDRATE_SERIAL0_=9600 -D_RTT_BAUDRATE_SERIAL1_=9600 -D_RTT_CONFIG_SERIAL0_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL1_=SERIAL_8N1 -D_RTT_ANALOG_REF_=0 -DMW_RTIO_SERIAL0 -DNANO_MOTOR_CARRIER
DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DEXT_MODE=1 -DONESTEPFCN=1 -DTERMFCN=1 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0
DEFINES_CUSTOM = 
DEFINES_OPTS = -DXCP_DAQ_SUPPORT -DXCP_CALIBRATION_SUPPORT -DXCP_TIMESTAMP_SUPPORT -DXCP_TIMESTAMP_BASED_ON_SIMULATION_TIME -DXCP_SET_MTA_SUPPORT -DEXTMODE_XCP_TRIGGER_SUPPORT -DXCP_MEM_BLOCK_1_SIZE=32 -DXCP_MEM_BLOCK_1_NUMBER=1 -DXCP_MEM_BLOCK_2_SIZE=56 -DXCP_MEM_BLOCK_2_NUMBER=1 -DXCP_MEM_BLOCK_3_SIZE=40 -DXCP_MEM_BLOCK_3_NUMBER=1 -DXCP_MEM_RESERVED_POOLS_TOTAL_SIZE=774 -DXCP_MEM_RESERVED_POOLS_NUMBER=2 -DXCP_MEM_DAQ_RESERVED_POOL_BLOCKS_NUMBER=3 -DXCP_MEM_DAQ_RESERVED_POOLS_NUMBER=1 -DXCP_MIN_EVENT_NO_RESERVED_POOL=1 -DXCP_MAX_CTO_SIZE=255 -DXCP_MAX_DTO_SIZE=65532 -DXCP_MAX_ODT_ENTRY_SIZE=255 -DEXTMODE_STATIC -DEXTMODE_STATIC_SIZE=8192 -DON_TARGET_WAIT_FOR_START=1 -DTID01EQ=0
DEFINES_SKIPFORSIL = -DXCP_CUSTOM_PLATFORM -DEXIT_FAILURE=1 -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -DSTACK_SIZE=64 -DRT
DEFINES_STANDARD = -DMODEL=hardwareModel_5 -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0

DEFINES = $(DEFINES_) $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/MW_arduinoI2C.cpp /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/MW_PWM.cpp /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/MW_PWMDriver.c /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/ArduinoPinHandleMap.cpp $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_mode.c $(START_DIR)/hardwareModel_5_ert_rtw/hardwareModel_5.c $(START_DIR)/hardwareModel_5_ert_rtw/hardwareModel_5_data.c $(START_DIR)/hardwareModel_5_ert_rtw/rtGetInf.c $(START_DIR)/hardwareModel_5_ert_rtw/rtGetNaN.c $(START_DIR)/hardwareModel_5_ert_rtw/rt_nonfinite.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_common.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_classic_trigger.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_standard.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_daq.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_calibration.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_fifo.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_transport.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_mem_default.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_drv_rtiostream.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/xcp_utils.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_frame_serial.c $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_param_default_serial.c /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/mw_samd_nvic.c /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoARM_M0plusScheduler.cpp $(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/src/m0m1m0plus_multitasking.c /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/src/platform_timer.c /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/src/rtiostream_serial_daemon.cpp $(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/xcp/src/sys_arch.c

MAIN_SRC = $(START_DIR)/hardwareModel_5_ert_rtw/ert_main.c

ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = MW_arduinoI2C.o io_wrappers.o MW_PWM.o MW_PWMDriver.o ArduinoPinHandleMap.o xcp_ext_mode.o hardwareModel_5.o hardwareModel_5_data.o rtGetInf.o rtGetNaN.o rt_nonfinite.o xcp_ext_common.o xcp_ext_classic_trigger.o xcp.o xcp_standard.o xcp_daq.o xcp_calibration.o xcp_fifo.o xcp_transport.o xcp_mem_default.o xcp_drv_rtiostream.o xcp_utils.o xcp_frame_serial.o xcp_ext_param_default_serial.o MW_ArduinoHWInit.o mw_samd_nvic.o arduinoARM_M0plusScheduler.o m0m1m0plus_multitasking.o platform_timer.o rtiostream_serial_daemon.o sys_arch.o

MAIN_OBJ = ert_main.o

ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(SSLIB_PATH)/MW_RebuildSrc_Comm.o $(SSLIB_PATH)/MW_RebuildSrc_Core.o

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_SKIPFORSIL = -MMD -mcpu=cortex-m0plus -fpermissive -DF_CPU=48000000L -DARDUINO_SAMD_NANO_33_IOT -DARDUINO_ARCH_SAMD -D__SAMD21G18A__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x8057 -DUSBCON -DUSB_MANUFACTURER=\""Arduino LLC\"" -DUSB_PRODUCT=\""Arduino NANO 33 IoT\"" -D_RUNONTARGETHARDWARE_BUILD_ -D_ROTH_NANO33_IOT_ -DCRYSTALLESS -DARDUINO_VIRTUAL_COM_PORT -DARDUINO_WIFI_LIB_NINA -DARDUINO_NUM_SERIAL_PORTS=2 -DARDUINO_ARM_CORTEX_M0_PLUS -DARDUINO_ARM
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_SKIPFORSIL) $(CFLAGS_BASIC)

#-----------
# Linker
#-----------

LDFLAGS_ = -L"$(SLIB_PATH)"
LDFLAGS_SKIPFORSIL = -T$(ARDUINO_SAMD_ROOT)/hardware/samd/$(SAMD_LIB_VERSION)/variants/nano_33_iot/linker_scripts/gcc/flash_with_bootloader.ld --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group -L$(ARDUINO_SAMD_ROOT)/tools/CMSIS/$(SAMD_CMSIS_VERSION)/CMSIS/Lib/GCC -l arm_cortexM0l_math

LDFLAGS += $(LDFLAGS_) $(LDFLAGS_SKIPFORSIL)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = -L"$(SLIB_PATH)"
SHAREDLIB_LDFLAGS_SKIPFORSIL = -T$(ARDUINO_SAMD_ROOT)/hardware/samd/$(SAMD_LIB_VERSION)/variants/nano_33_iot/linker_scripts/gcc/flash_with_bootloader.ld --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group -L$(ARDUINO_SAMD_ROOT)/tools/CMSIS/$(SAMD_CMSIS_VERSION)/CMSIS/Lib/GCC -l arm_cortexM0l_math

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_) $(SHAREDLIB_LDFLAGS_SKIPFORSIL)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_SKIPFORSIL = -MMD -mcpu=cortex-m0plus -fpermissive -DF_CPU=48000000L -DARDUINO_SAMD_NANO_33_IOT -DARDUINO_ARCH_SAMD -D__SAMD21G18A__ -mthumb -DUSB_VID=0x2341 -DUSB_PID=0x8057 -DUSBCON -DUSB_MANUFACTURER=\""Arduino LLC\"" -DUSB_PRODUCT=\""Arduino NANO 33 IoT\"" -D_RUNONTARGETHARDWARE_BUILD_ -D_ROTH_NANO33_IOT_ -DCRYSTALLESS -DARDUINO_VIRTUAL_COM_PORT -DARDUINO_WIFI_LIB_NINA -DARDUINO_NUM_SERIAL_PORTS=2 -DARDUINO_ARM_CORTEX_M0_PLUS -DARDUINO_ARM
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_SKIPFORSIL) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = -L"$(SLIB_PATH)"
CPP_LDFLAGS_SKIPFORSIL = -T$(ARDUINO_SAMD_ROOT)/hardware/samd/$(SAMD_LIB_VERSION)/variants/nano_33_iot/linker_scripts/gcc/flash_with_bootloader.ld --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group -L$(ARDUINO_SAMD_ROOT)/tools/CMSIS/$(SAMD_CMSIS_VERSION)/CMSIS/Lib/GCC -l arm_cortexM0l_math

CPP_LDFLAGS += $(CPP_LDFLAGS_) $(CPP_LDFLAGS_SKIPFORSIL)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = -L"$(SLIB_PATH)"
CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL = -T$(ARDUINO_SAMD_ROOT)/hardware/samd/$(SAMD_LIB_VERSION)/variants/nano_33_iot/linker_scripts/gcc/flash_with_bootloader.ld --specs=nano.specs --specs=nosys.specs -mcpu=cortex-m0plus -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--start-group -L$(ARDUINO_SAMD_ROOT)/tools/CMSIS/$(SAMD_CMSIS_VERSION)/CMSIS/Lib/GCC -l arm_cortexM0l_math

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_) $(CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild postbuild download execute


all : build postbuild
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	echo "### Successfully generated all binary outputs."


prebuild : 


postbuild : $(PRODUCT)
	echo "### Invoking postbuild tool "Binary Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_BIN)
	echo "### Done invoking postbuild tool."


download : postbuild


execute : download
	echo "### Invoking postbuild tool "Execute" ..."
	$(EXECUTE) $(EXECUTE_FLAGS)
	echo "### Done invoking postbuild tool."


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS) $(MAIN_OBJ)
	echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(OBJS) $(MAIN_OBJ) $(LIBS) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : %.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/hardwareModel_5_ert_rtw/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/simulink/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/simulink/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.C
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_arduinoI2C.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/MW_arduinoI2C.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


io_wrappers.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_PWM.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/MW_PWM.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_PWMDriver.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/MW_PWMDriver.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ArduinoPinHandleMap.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/ArduinoPinHandleMap.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xcp_ext_mode.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_mode.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ert_main.o : $(START_DIR)/hardwareModel_5_ert_rtw/ert_main.c
	$(CC) $(CFLAGS) -o "$@" "$<"


hardwareModel_5.o : $(START_DIR)/hardwareModel_5_ert_rtw/hardwareModel_5.c
	$(CC) $(CFLAGS) -o "$@" "$<"


hardwareModel_5_data.o : $(START_DIR)/hardwareModel_5_ert_rtw/hardwareModel_5_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/hardwareModel_5_ert_rtw/rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/hardwareModel_5_ert_rtw/rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/hardwareModel_5_ert_rtw/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_ext_common.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_common.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_ext_classic_trigger.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_classic_trigger.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_standard.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_standard.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_daq.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_daq.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_calibration.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/protocol/src/xcp_calibration.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_fifo.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_fifo.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_transport.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_transport.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_mem_default.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_mem_default.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_drv_rtiostream.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/platform/default/xcp_drv_rtiostream.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_utils.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/common/xcp_utils.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_frame_serial.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/server/transport/src/xcp_frame_serial.c
	$(CC) $(CFLAGS) -o "$@" "$<"


xcp_ext_param_default_serial.o : $(MATLAB_ROOT)/toolbox/coder/xcp/src/target/ext_mode/src/xcp_ext_param_default_serial.c
	$(CC) $(CFLAGS) -o "$@" "$<"


MW_ArduinoHWInit.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mw_samd_nvic.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinobase/src/mw_samd_nvic.c
	$(CC) $(CFLAGS) -o "$@" "$<"


arduinoARM_M0plusScheduler.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoARM_M0plusScheduler.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


m0m1m0plus_multitasking.o : $(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/scheduler/src/m0m1m0plus_multitasking.c
	$(CC) $(CFLAGS) -o "$@" "$<"


platform_timer.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/src/platform_timer.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtiostream_serial_daemon.o : /root/Documents/MATLAB/SupportPackages/R2023b/toolbox/target/supportpackages/arduinotarget/src/rtiostream_serial_daemon.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sys_arch.o : $(MATLAB_ROOT)/toolbox/target/shared/armcortexmbase/xcp/src/sys_arch.c
	$(CC) $(CFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### ASFLAGS = $(ASFLAGS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### OBJCOPYFLAGS_BIN = $(OBJCOPYFLAGS_BIN)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.dep
	$(ECHO) "### Deleted all derived files."


