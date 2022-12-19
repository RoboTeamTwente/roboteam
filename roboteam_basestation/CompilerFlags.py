# This file sets additional flags that cannot be set from platformio.ini itself
# these can be for the compiler or for the linker
import configparser
import subprocess
import re

# load build environment
print("\n==================== CompilerFlags.py ====================")
l = lambda *args, **kwargs : print("[CompilerFlags.py]", *args, **kwargs)

Import("env")

# read settings from platform.ini to also include
conf = configparser.ConfigParser()
conf.read("platformio.ini")
opt = conf.get("env:nucleo_f767zi", "optimization")

git_branch_name = subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).decode('ascii').strip()
git_commit_date = subprocess.check_output(['git', 'show', '-s', '--date=format:%d/%m/%Y', '--format=%cd', 'HEAD']).decode('ascii').strip()
git_commit_hash = subprocess.check_output(['git', 'show', '-s', '--format=%h', 'HEAD']).decode('ascii').strip()
git_commit_msg  = subprocess.check_output(['git', 'show', '-s', '--format=%s', 'HEAD']).decode('ascii').strip()

git_string = f"{git_branch_name} {git_commit_hash} | {git_commit_date} | {git_commit_msg}"
git_string = re.sub('-', '_', git_string)
git_string = re.sub(r'[^a-zA-Z0-9 /_]', '', git_string)
l(git_string)


# read settings from platform.ini to also include
conf = configparser.ConfigParser()
conf.read("platformio.ini")
opt = conf.get("env:nucleo_f767zi", "optimization")

# compiler settings
env.Append(
  CCFLAGS=[
    f"-D__GIT_STRING__={git_string}",
    f"-D__GIT_DEVELOPMENT__=\"{[0, 1][git_branch_name == 'development']}\"",
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

print("\n")