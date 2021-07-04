import math



funcDeclGet = """
    @staticmethod
    def get_%s(payload):
%s
"""

funcDeclSet = """
    @staticmethod
    def set_%s(payload, %s):
%s

"""


# Capitalizes the first letter of a word
def upperFirst(word):
	return word[0].capitalize() + word[1:]
# RobotCommand => rc
def CamelCaseToAbbrev(word):
	return ''.join([letter for letter in word if letter.isupper()]).lower()
# RobotCommand => _ROBOT_COMMAND
def CamelCaseToUpper(word):
	return ''.join(['_'*char.isupper() + char.upper() for char in word])


### Returns a string of 0bxxxxxxxx where x=1 if x in range [start, stop), else 0. If reverse=True, swap 0 and 1
# bitString(1, 2, reverse=False) = 0b01000000
# bitString(5, 6, reverse=True ) = 0b11111011
def bitString(start, stop, reverse = False):
	set1 = list(range(start, stop))
	bitstring = "0b" + ''.join(["%d" % ((i in set1) ^ reverse) for i in range(8)])
	return bitstring
	

# Return either <<, >>, or nothing, depending on left vs right	
def shift(left, right):
	if left == right: return ""
	if left < right: return " >> %d" % (right-left)
	if right < left: return " << %d" % (left-right)


# Change number of bits to required integer size
def getType(nBits, _range):
	_type = "bool"
	if  1 < nBits: _type = "uint32_t"
	if 32 < nBits: _type = "uint64_t"
	return _type


# Returns a and b in the formula y = ax+b
def getConversion(nBits, _range):
		vMin, vMax = _range
		vTotal = vMax - vMin
		return vTotal / 2**nBits, vMin



# class RobotCommand:
#     self.header = 0;          # Header byte indicating the type of packet
#     self.id = 0;              # Id of the robot
#     self.messageId = 0;       # Id of the message
#     ....
def toClass(packet, variables):
	strClass = "class %s:\n" % packet
	for variable, nBits,  _range, desc in variables:
		strVar = "    %s = 0;" % variable # Add variable name
		strClass += strVar.ljust(30) + "# %s\n" % desc 	# Add description
	strClass += "\n"
	return strClass


# RobotCommand, [[var], [var]] =>
# [---0--] [---1--] [---2--] [---3--] [---4--] [---5--] [---6--] [---7--] [---8--] [---9--] [--10--]
# 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
def toStructure(packet, variables):
	# Calculate number of bytes needed to store all variables. ceil(total number of bits / 8)
	nBytes = math.ceil(sum([nBits for (	_, nBits, _, _) in variables]) / 8)
	# Create [---0--] [---1--] [---2--] .......
	struct = ' '.join(["[%s--]" % (("%d"%i).rjust(4,"-")) for i in range(nBytes)])
	# Keep track of location in the packet
	at = 0
	for variable, nBits, _, desc in variables:
		set1 = list(range(at, at+nBits)) # Indices of active bits
		struct += "\n"	
		for i in range(nBytes * 8): # For all bits
			if i != 0 and i % 8 == 0: struct += " " # Add whitespace between bytes
			struct += "1" if i in set1 else "-" # Add '1' if bit active, '-' if inactive
		struct += " %s" % variable # Add variable name at the end
		at += nBits
	return struct


def toGetters(packetName, packets):
	current_bit = 0        # Keep track of the active bit in the packet 
	functions_string = ""  # Variable to hold all the functions

	indent = "        " # Python function indent

	# For each entry in a packet, e.g.  ["id", 4,  None, "Id of the robot"],
	for variable, n_bits_in_variable, _range, _ in packets[packetName]:
		
		n_bits_remaining = n_bits_in_variable
		# Get variable type. Either bool, uint32_t or uint64_t
		variable_type = getType(n_bits_in_variable, _range) # Either bool, uint32_t, or uint64_t
		float_conversion = _range is not None

		returnValue = ""

		if variable_type == "bool":
			# Find the byte index, and the bit index within that byte
			atByte, atBit = current_bit // 8, current_bit % 8
			returnValue = f"{indent}return (payload[{atByte}] & {bitString(atBit, atBit+1)}) > 0"

		if variable_type in ["uint32_t", "uint64_t"]:
			# Collect all the bit operations, all the << and | and & and whatever needed to get the correct bits
			operations = []
			while 0 < n_bits_remaining:
				# Find the byte index in the packet, and the bit index within that byte
				atByte, atBit = current_bit // 8, current_bit % 8
				# Calculate the number of bits to process from the current byte. Always 8 or less
				bitsFromByte = min(n_bits_remaining, 8-atBit)
				# Figure out which bits we need from the byte
				bStart, bStop = atBit, atBit + bitsFromByte
				# Generate bitshift mask, except if the entire byte is used
				mask = "" if bitsFromByte == 8 else " & %s" % bitString(bStart, bStop)
				# Figure out how much we need to shift this baby in total
				# e.g. uint16_t and 0b1110000: 16 to the left, 5 to the right => 11 to the left
				shiftRight = (8-bStop)
				shiftLeft = n_bits_remaining - bitsFromByte

				operation = f"((payload[{atByte}]{mask}){shift(shiftLeft, shiftRight)})"
				operations.append(operation)

				n_bits_remaining -= bitsFromByte
				current_bit += bitsFromByte

			# Concatenate all bit operations
			bitwise_operations_string = ' | '.join(operations)

			# Create the conversion from float to integer, if needed
			conversion_string = ""
			if float_conversion:
				# Formula : y = ax + b => x = (y - b) / a
				a, b = getConversion(n_bits_in_variable, _range)
				subtract_string = f"{variable}" if b == 0 else f"(_{variable} {-b:+.16f})"
				division_string = f" / {a:.16f}"
				conversion_string += f"{indent}{variable} = int({subtract_string}{division_string})\n"
			
			# No range given, so no conversion to float needed
			if _range is None:
				returnValue = f"{indent}return {bitwise_operations_string}"
			# Range given. Do integer => float conversion
			else: 
				a, b = getConversion(n_bits_in_variable, _range) # y = ax + b
				print(variable, a ,b)
				returnValue = f"{indent}{variable} = {bitwise_operations_string}\n"
				returnValue+= f"{indent}return ({variable} * {a:.16f}) + {b:.16f}"

		func = funcDeclGet % (variable, returnValue)
		functions_string += "    " + func
		current_bit += n_bits_remaining

	return functions_string

def toSetters(packetName, packets):
	current_bit = 0
	functionsString = ""

	indent = "        " # Python function indent

	# For each entry such as ["header", 8,  None, "Header byte indicating the type of packet"],
	for variable, nBits, _range, _ in packets[packetName]:
		n_bits_in_variable = nBits

		variable_type = getType(nBits, _range) # Either bool, uint32_t, or uint64_t
		float_conversion = _range is not None

		# Create the conversion from float to integer, if needed
		conversion_string = ""
		if float_conversion:
			# Formula : y = ax + b => x = (y - b) / a
			a, b = getConversion(n_bits_in_variable, _range)
			subtract_string = f"_{variable}" if b == 0 else f"(_{variable} {-b:+.16f})"
			division_string = f" / {int(a)}" if int(a) == float(a) else f" / {a:.16f}"
			conversion_string += f"{indent}{variable} = int({subtract_string}{division_string})\n"

		# Create all bitwise operations
		bitwise_operations = []
		while 0 < nBits:
			# Find the byte index, and the bit index within that byte
			atByte, atBit = current_bit // 8, current_bit % 8
			bitsInByte = min(nBits, 8 - atBit)
			# Figure out which bits we need from the byte
			bStart, bStop = atBit, atBit + bitsInByte
			# Generate bitmask, except if the entire byte is used
			bitmask = "" if bitsInByte == 8 else " & %s" % bitString(bStart, bStop)
			# Generate inverse bitmask
			bitmask_inverted = " & %s" % bitString(bStart, bStop, True)

			shiftLeft = 8 - bStop
			shiftRight = max(0, nBits - bitsInByte)

			inverse = ""
			if bitsInByte < 8:
				inverse = f" | (payload[{atByte}]{bitmask_inverted})"

			# Add shifting of variable if needed
			shiftVariable = f"({variable}{shift(shiftLeft, shiftRight)})"
			if shiftLeft == 0 and shiftRight == 0 : shiftVariable = variable
			
			# Add masking to variable if needed
			maskVariable = f"({shiftVariable}{bitmask})"
			if bitsInByte == 8 : maskVariable = shiftVariable

			operation = f"{indent}payload[{atByte}] = {maskVariable}{inverse}"
			bitwise_operations.append(operation)

			nBits -= bitsInByte
			current_bit += bitsInByte

		operations_string = conversion_string
		operations_string += '\n'.join(bitwise_operations)

		# convert 'theta' to '_theta' if a float conversion occured
		_variable = "_"+variable if float_conversion else variable 
		
		func = funcDeclSet % (variable, _variable, operations_string)
		functionsString += "    " + func
		current_bit += nBits

	return functionsString

def toDecode(packet, variables):
	f = "    def decode(self, payload):\n"
	for variable, nBits, _, _ in variables:
		f += ("        self.%s"%(variable)).ljust(30) + "= %s.get_%s(payload)\n" % (packet, variable)
	return f

def toEncode(packet, variables):
	f =  "    def encode(self):\n"
	f += "        payload = np.zeros(BaseTypes.PACKET_SIZE%s, dtype=np.uint8)\n" % CamelCaseToUpper(packet)
	for variable, nBits, _, _ in variables:
		f += "        %s.set_%s(payload, self.%s)\n" % (packet, variable.ljust(20), variable)
	f += "        return payload\n"
	return f

def addPrintBitString():
	return """
	def printBitString(self):
		payload = self.encode()
		for i in range(len(payload)):
			print(format(payload[i], '08b'), end=" ")
		print()
	"""
