#This file is generated by VisualGDB.
#It contains GCC settings automatically derived from the board support package (BSP).
#DO NOT EDIT MANUALLY. THE FILE WILL BE OVERWRITTEN. 
#Use VisualGDB Project Properties dialog or modify Makefile or per-configuration .mak files instead.

#In order to build this project manually (outside VisualGDB), please provide TOOLCHAIN_ROOT, BSP_ROOT, EFP_BASE and TESTFW_BASE variables via Environment or Make command line
#Embedded toolchain
CC := $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc.exe
CXX := $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-g++.exe
LD := $(CXX)
AR := $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-ar.exe
OBJCOPY := $(TOOLCHAIN_ROOT)/bin/arm-none-eabi-objcopy.exe

#Additional flags
PREPROCESSOR_MACROS += ARM_MATH_CM7 flash_layout STM32F745VE STM32F745xx USE_HAL_DRIVER USE_USB_FS 
INCLUDE_DIRS += . $(BSP_ROOT)/STM32F7xxxx/STM32F7xx_HAL_Driver/Inc $(BSP_ROOT)/STM32F7xxxx/STM32F7xx_HAL_Driver/Inc/Legacy $(BSP_ROOT)/STM32F7xxxx/CMSIS_HAL/Core/Include $(BSP_ROOT)/STM32F7xxxx/CMSIS_HAL/Device/ST/STM32F7xx/Include $(BSP_ROOT)/STM32F7xxxx/CMSIS_HAL/Include $(BSP_ROOT)/STM32F7xxxx/CMSIS_HAL/RTOS2/Include $(BSP_ROOT)/STM32F7xxxx/STM32_USB_Device_Library/Core/Inc $(BSP_ROOT)/STM32F7xxxx/STM32_USB_Device_Library/Class/CDC/Inc
LIBRARY_DIRS += 
LIBRARY_NAMES += 
ADDITIONAL_LINKER_INPUTS += 
MACOS_FRAMEWORKS += 
LINUX_PACKAGES += 

CFLAGS += 
CXXFLAGS += 
ASFLAGS += 
LDFLAGS += --specs=nano.specs -u _printf_float --specs=nosys.specs
COMMONFLAGS += -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
LINKER_SCRIPT := $(BSP_ROOT)/STM32F7xxxx/LinkerScripts/STM32F745VE_flash.lds

