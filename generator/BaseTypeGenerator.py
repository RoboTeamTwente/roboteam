import os
import math
from packets import packets
from datetime import datetime


# RobotCommand => _ROBOT_COMMAND
def CamelCaseToUpper(word):
	return ''.join(['_'*char.isupper() + char.upper() for char in word])



class BaseTypeGenerator:

	def generate(self, packets):

		codewords = [
						  "0b00001111", "0b00110011", "0b00111100",
			"0b01010101", "0b01011010", "0b01100110", "0b01101001",
			"0b10010110", "0b10011001", "0b10100101", "0b10101010",
			"0b11000011", "0b11001100", "0b11110000", "0b11111111"
		]

		if len(codewords) < len(packets.keys()):
			print("[Error] More packets than codewords. Add more codewords. Remove packets! DO SOMETHING!!!")
			print("[Error] If you're really desparate and in a hurry, then paste the array below.")
			print("[Error] But these are not good codewords!! Might as well have no codewords.")
			codewords = ["0b" + "{0:b}".format(20 + i*5).zfill(8) for i in range(len(packets))]
			print(codewords)
			exit()


		timestamp = datetime.now()
		timestamp_string = timestamp.strftime("%B %d %Y, %H:%M:%S")
		c = self.comment()
		
		file_string = ""

		file_string += f"{c} AUTOGENERATED. Run generator/main.py to regenerate\n"
		file_string += f"{c} Generated on %s\n\n" % timestamp.strftime("%B %d %Y, %H:%M:%S")

		file_string += f"{c} Warning : Check the unicode table before assigning a byte, to make sure that the byte isn't used for anything special : https://unicode-table.com/\n"
		file_string += f"{c} For example, don't use the following bytes\n"
		file_string += f"{c} 0b00000000 : The null-terminator, used to signal the end of strings / arrays / etc.\n"
		file_string += f"{c} 0b00001010 : The byte for newline, used for line termination.\n"
		file_string += f"\n"

		file_string += self.to_begin()

		for iPacket, packet_name in enumerate(packets.keys()):
			variables = packets[packet_name]
			total_bits = sum([variable[1] for variable in variables])
			total_bytes = math.ceil(total_bits / 8)
			
			PACKET_NAME = CamelCaseToUpper(packet_name)

			VARIABLE_NAME = f"PACKET_TYPE{PACKET_NAME}".ljust(60)
			file_string += f"{self.to_constant(VARIABLE_NAME, codewords[iPacket])} {c} {str(int(codewords[iPacket], 2))} \n"

			VARIABLE_NAME = f"PACKET_SIZE{PACKET_NAME}".ljust(60)
			file_string += self.to_constant(VARIABLE_NAME, total_bytes) + "\n"

			for variable, _, _range, _ in variables:
				if _range is None: continue
				VARIABLE_NAME = CamelCaseToUpper(variable)
				file_string += self.to_constant(f"PACKET_RANGE{PACKET_NAME}_{VARIABLE_NAME}_MIN".ljust(60), f"{_range[0]:.16f}".rstrip('0')) + "\n"
				file_string += self.to_constant(f"PACKET_RANGE{PACKET_NAME}_{VARIABLE_NAME}_MAX".ljust(60), f"{_range[1]:.16f}".rstrip('0')) + "\n"

			file_string += "\n"

		file_string += self.to_end()

		return file_string

	def begin_block_comment(self):
		raise NotImplementedError()
	def end_block_comment(self):
		raise NotImplementedError()
	def comment(self):
		raise NotImplementedError()

	def to_begin(self):
		return ""
	def to_end(self):
		return ""

	def to_constant(self, variable_name, value):
		raise NotImplementedError()

class C_BaseTypeGenerator(BaseTypeGenerator):
	def begin_block_comment(self):
		return "/*"
	def end_block_comment(self):
		return "*/"
	def comment(self):
		return "//"

	def to_begin(self):
		begin_string = ""
		begin_string += "#ifndef __BASETYPES_H\n"
		begin_string += "#define __BASETYPES_H\n"
		begin_string += "\n"
		return begin_string

	def to_end(self):
		return "#endif /*__BASETYPES_H*/"

	def to_constant(self, variable_name, value):
		return f"#define {variable_name} {value}"

class Python_BaseTypeGenerator(BaseTypeGenerator):
	def begin_block_comment(self):
		return '"""'
	def end_block_comment(self):
		return '"""'
	def comment(self):
		return "#"

	def to_constant(self, variable_name, value):
		return f"{variable_name} = {value}"



# codewords = [
# 	              "0b00001111", "0b00110011", "0b00111100",
# 	"0b01010101", "0b01011010", "0b01100110", "0b01101001",
# 	"0b10010110", "0b10011001", "0b10100101", "0b10101010",
# 	"0b11000011", "0b11001100", "0b11110000", "0b11111111"
# ]

# if len(codewords) < len(packets.keys()):
# 	print("[Error] More packets than codewords. Add more codewords. Remove packets! DO SOMETHING!!!")
# 	print("[Error] If you're really desparate and in a hurry, then paste the array below.")
# 	print("[Error] But these are not good codewords!! Might as well have no codewords.")
# 	codewords = ["0b" + "{0:b}".format(20 + i*5).zfill(8) for i in range(len(packets))]
# 	print(codewords)
# 	exit()


# ############### Generate BaseTypes.h ###############
# # Set of codewords with hamming distance 4. CRC in the SX1280 should already catch bit errors, buuuuut just to be sure
# # Don't want a robot to go haywire because it interpreted another message as a RobotCommand
# # Table taken from this paper : "The VFAT3-Comm-Port: a complete communication port for front-end ASICs intended for use within the high luminosity radiation environments of the LHC"
# # I have no idea what that title means. It's complete gibberish. But whatever, it has nice codeword tables

# # Warning : Check the unicode table before assigning a byte, to make sure that the byte isn't used for anything special : https://unicode-table.com/
# # For example, don't use the following bytes
# #      0b00000000 : The null-terminator, used to signal the end of strings / arrays / etc.
# #      0b00001010 : The byte for newline, used for line termination.

# filename = "BaseTypes.h"
# file = open("generated_c/" + filename, "w")
# timestamp = datetime.now()

# file.write("// AUTOGENERATED. Run generator/main.py to regenerate\n")
# file.write("// Generated on %s\n\n" % timestamp.strftime("%B %d %Y, %H:%M:%S"))

# file.write("#ifndef __BASETYPES_H\n")
# file.write("#define __BASETYPES_H\n")
# file.write("\n")

# file.write("// Warning : Check the unicode table before assigning a byte, to make sure that the byte isn't used for anything special : https://unicode-table.com/\n")
# file.write("// For example, don't use the following bytes\n")
# file.write("// 0b00000000 : The null-terminator, used to signal the end of strings / arrays / etc.\n")
# file.write("// 0b00001010 : The byte for newline, used for line termination.\n")
# file.write("\n")

# packet_sizes = []
# packet_types = []
# packet_ranges = []

# for iPacket, packetName in enumerate(packets.keys()):
# 	variables = packets[packetName]
# 	total_bits = sum([variable[1] for variable in variables])
# 	total_bytes = math.ceil(total_bits / 8)
	
# 	PACKET_NAME = Generator.CamelCaseToUpper(packetName)
# 	packet_type = f"    PACKET_TYPE{PACKET_NAME}".ljust(50) + " = " + codewords[iPacket] + " /* " + str(int(codewords[iPacket], 2)) + " */"
# 	packet_size = f"    PACKET_SIZE{PACKET_NAME}".ljust(50) + " = " + str(total_bytes)
	
# 	packet_types.append(packet_type)
# 	packet_sizes.append(packet_size)

# 	for variable, _, _range, _ in variables:
# 		if _range is None: continue
# 		VARIABLE = Generator.CamelCaseToUpper(variable)
# 		packet_range_min = f"#define PACKET_RANGE{PACKET_NAME}_{VARIABLE}_MIN".ljust(60) + f"{_range[0]:+.16f}".rstrip('0')
# 		packet_range_max = f"#define PACKET_RANGE{PACKET_NAME}_{VARIABLE}_MAX".ljust(60) + f"{_range[1]:+.16f}".rstrip('0')
# 		packet_ranges.append(packet_range_min)
# 		packet_ranges.append(packet_range_max)

# file.write("typedef enum _PACKET_TYPE {\n")
# file.write(",\n".join(packet_types) + "\n")
# file.write("} PACKET_TYPE;\n")
# file.write("\n")
# file.write("typedef enum _PACKET_SIZE {\n")
# file.write(",\n".join(packet_sizes) + "\n")
# file.write("} PACKET_SIZE;\n")
# file.write("\n")
# file.write("\n".join(packet_ranges) + "\n")
# file.write("\n")
# file.write("# endif /*__BASETYPES_H*/")
# file.write("\n")

# file.close()

# # shutil.copy("generated/%s" % filename, "../include/%s" % filename)
# print("Generated file BaseTypes.h")



# ############### Generate BaseTypes.py ###############
# # Set of codewords with hamming distance 4. CRC in the SX1280 should already catch bit errors, buuuuut just to be sure
# # Don't want a robot to go haywire because it interpreted another message as a RobotCommand
# # Table taken from this paper : "The VFAT3-Comm-Port: a complete communication port for front-end ASICs intended for use within the high luminosity radiation environments of the LHC"
# # I have no idea what that title means. It's complete gibberish. But whatever, it has nice codeword tables

# # Warning : Check the unicode table before assigning a byte, to make sure that the byte isn't used for anything special : https://unicode-table.com/
# # For example, don't use the following bytes
# #      0b00000000 : The null-terminator, used to signal the end of strings / arrays / etc.
# #      0b00001010 : The byte for newline, used for line termination.

# filename = "BaseTypes.py"
# file = open("generated_python/" + filename, "w")
# timestamp = datetime.now()

# file.write("# AUTOGENERATED. Run generator/main.py to regenerate\n")
# file.write("# Generated on %s\n\n" % timestamp.strftime("%B %d %Y, %H:%M:%S"))

# file.write("# Warning : Check the unicode table before assigning a byte, to make sure that the byte isn't used for anything special : https://unicode-table.com/\n")
# file.write("# For example, don't use the following bytes\n")
# file.write("# 0b00000000 : The null-terminator, used to signal the end of strings / arrays / etc.\n")
# file.write("# 0b00001010 : The byte for newline, used for line termination.\n")
# file.write("\n")

# packet_sizes = []
# packet_types = []
# packet_ranges = []

# for iPacket, packetName in enumerate(packets.keys()):
# 	variables = packets[packetName]
# 	total_bits = sum([variable[1] for variable in variables])
# 	total_bytes = math.ceil(total_bits / 8)
	
# 	PACKET_NAME = Generator.CamelCaseToUpper(packetName)
# 	packet_type = f"PACKET_TYPE{PACKET_NAME}".ljust(50) + " = " + codewords[iPacket] + " # " + str(int(codewords[iPacket], 2))
# 	packet_size = f"PACKET_SIZE{PACKET_NAME}".ljust(50) + " = " + str(total_bytes)
	
# 	packet_types.append(packet_type)
# 	packet_sizes.append(packet_size)

# 	for variable, _, _range, _ in variables:
# 		if _range is None: continue
# 		VARIABLE = Generator.CamelCaseToUpper(variable)
# 		packet_range_min = f"PACKET_RANGE{PACKET_NAME}_{VARIABLE}_MIN".ljust(60) + f"= {_range[0]:+.16f}".rstrip('0')
# 		packet_range_max = f"PACKET_RANGE{PACKET_NAME}_{VARIABLE}_MAX".ljust(60) + f"= {_range[1]:+.16f}".rstrip('0')
# 		packet_ranges.append(packet_range_min)
# 		packet_ranges.append(packet_range_max)
	

# file.write("\n".join(packet_types) + "\n")
# file.write("\n")
# file.write(",\n".join(packet_sizes) + "\n")
# file.write("\n")
# file.write("\n".join(packet_ranges) + "\n")
# file.write("\n")

# file.close()

# # shutil.copy("generated/%s" % filename, "../include/%s" % filename)
# print("Generated file BaseTypes.py")



if __name__ == "__main__":
	print("BaseTypesGenerator.py")
	gc = C_BaseTypeGenerator()
	gp = Python_BaseTypeGenerator()

	print("\n")
	print(gc.generate(packets))

	print("\n")
	print(gp.generate(packets))
