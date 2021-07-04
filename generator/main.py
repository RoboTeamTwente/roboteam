import os
from datetime import datetime
import math
import shutil

import Generator
import BaseTypeGenerator
from packets import packets

generator_c = Generator.C_Generator()
generator_python = Generator.Python_Generator()

os.makedirs("generated_c", exist_ok=True)
os.makedirs("generated_python", exist_ok=True)

for packet_name in packets:
	variables = packets[packet_name]

	filename = f"{packet_name}.h"
	with open(os.path.join("generated_c", filename), "w") as file:
		file.write(generator_c.generate(packet_name, variables))
	print(f"Generated file {filename}")

	filename = f"{packet_name}.py"
	with open(os.path.join("generated_python", filename), "w") as file:
		file.write(generator_python.generate(packet_name, variables))
	print(f"Generated file {filename}")



basetypegenerator_c = BaseTypeGenerator.C_BaseTypeGenerator()
basetypegenerator_python = BaseTypeGenerator.Python_BaseTypeGenerator()

filename = "BaseTypes.h"
with open(os.path.join("generated_c", filename), "w") as file:
	file.write(basetypegenerator_c.generate(packets))
print(f"Generated file {filename}")

filename = "BaseTypes.py"
with open(os.path.join("generated_python", filename), "w") as file:
	file.write(basetypegenerator_python.generate(packets))
print(f"Generated file {filename}")




for file in os.listdir("generated_c"):
	shutil.move(f"generated_c/{file}", f"../include/{file}")

for file in os.listdir("generated_python"):
	shutil.move(f"generated_python/{file}", f"../python/{file}")










