# This file sets additional flags that cannot be set from platformio.ini itself
# these can be for the compiler or for the linker
import configparser

# load build environment
Import("env")

# read settings from platform.ini to also include
conf = configparser.ConfigParser()
conf.read("platformio.ini")
opt = conf.get("env:nucleo_f767zi", "optimization")

# compiler settings
env.Append(
  CCFLAGS=[
    opt,
    # "-std=c++11",
    "-g3",
    "-Wall",
    # "-Wextra",
    # "-Wconversion",
    # "-pedantic",
    "-mcpu=cortex-m7",
    "-mfloat-abi=hard",
    "-mfpu=fpv5-sp-d16",
    "-D ARM_MATH_CM7",
    "-DARM_MATH_MATRIX_CHECK",
    "-D __FPU_PRESENT",
    "-mthumb",
    "-mthumb-interwork",
    "-ffunction-sections",
    "-fdata-sections",
    "-fmessage-length=0",
    "-specs=nosys.specs",
    "-specs=nano.specs",
    "-DUSE_HAL_DRIVER",
    "-DSTM32F767xx",
  ]

)

# linker settings
env.Append(
  LINKFLAGS=[
    opt,
    "-mfloat-abi=hard",
	  "-mfpu=fpv5-sp-d16",
    "-Wl,-u,_printf_float,-u,_scanf_float"
  ]
)