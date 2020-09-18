import os
import shutil

# Migrate all folders except Core and DRIVERS
directories = [d for d in os.listdir('.') if os.path.isdir(d) and not d.startswith('.')]
directories.remove("Core")
directories.remove("Drivers")

print(directories)
# Copy all directories to both Core/Inc and Core/Src
for d in directories:
	Inc = os.path.join("Core", "Inc", d)
	Src = os.path.join("Core", "Src", d)
	shutil.rmtree(Inc, ignore_errors=True)
	shutil.rmtree(Src, ignore_errors=True)
	shutil.copytree(d, Inc)
	shutil.copytree(d, Src)

# Remove all not .h files from Core/Inc
for _dir, _subdirs, _files in os.walk("./Core/Inc"):
	for file in _files:
		if not file.endswith(".h"):
			os.remove(os.path.join(_dir, file))

# Remove all not .c files from Core/Src	ss		
for _dir, _subdirs, _files in os.walk("./Core/Src"):
	for file in _files:
		if not file.endswith(".c"):
			os.remove(os.path.join(_dir, file))

# Remove all empty directories
for _dir, _subdirs, _files in os.walk("./Core"):
	if len(_subdirs) + len(_files) == 0:
		os.rmdir(_dir)

# Find all directories with header files
includes_dirs = []
for _dir, _subdirs, _files in os.walk("./Core/Inc"):
	if 0 < len(_files):
		includes_dirs.append(_dir)

### Generate platformio file
with open("platformio.ini", "w") as file:
	file.write("""\
[platformio]
include_dir = Core/Inc
src_dir = Core/Src

[env:nucleo_f767zi]
platform = ststm32
board = nucleo_f767zi
framework = stm32cube

build_flags = 
""")
	for d in includes_dirs:
		file.write("\t-I %s\n" % d)


# files = []
# for _dir, _subdirs, _files in os.walk("./Core/Src"):
# 	files += [os.path.join(_dir, f) for f in _files if not f.endswith(".c")]
# print(files)

