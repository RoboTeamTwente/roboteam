import math



funcDeclGet = """static inline %s %s_get_%s(%s *%s){
%s
}\n"""

funcDeclSet = """static inline void %s_set_%s(%s *%s, %s %s){
%s
}\n"""


def upperFirst(word):
	return word[0].capitalize() + word[1:]

def bitString(start, stop, reverse = False):
	set1 = list(range(start, stop))
	return "0b" + ''.join(["%d" % ((i in set1) ^ reverse) for i in range(8)])
	
def shift(left, right):
	if left == right: return ""
	if left < right: return " >> %d" % (right-left)
	if right < left: return " << %d" % (left-right)

# RobotCommand => rc
def CamelCaseToAbbrev(word):
	return ''.join([letter for letter in word if letter.isupper()]).lower()

# RobotCommand => _ROBOT_COMMAND
def CamelCaseToUpper(word):
	return ''.join(['_'*char.isupper() + char.upper() for char in word])

def getTypes(nBits, _range):
	type1 = "bool"
	if  1 < nBits: type1 = "uint8_t"
	if  8 < nBits: type1 = "uint16_t"
	if 16 < nBits: type1 = "uint32_t"
	if 32 < nBits: type1 = "uint64_t"

	if _range is None : return [type1, type1]
	else :				return [type1, "float"]

def getConversionToFloat(nBits, _range):
		vMin, vMax = _range
		vTotal = vMax - vMin
		return vTotal / 2**nBits, vMin

def getConversionToInt(nBits, _range):
		vMin, vMax = _range
		vTotal = vMax - vMin
		return 2**nBits / vTotal, vMin


def toHeaderGuard(packet):
	return "_%s_H" % CamelCaseToUpper(packet)

def toStructPayload(packet):
	struct = "typedef struct _%sPayload {\n\tuint8_t payload[PACKET_SIZE%s];\n} %sPayload;"
	struct %= (packet, CamelCaseToUpper(packet), packet)
	return struct

def toStruct(packet, variables):
	struct = "typedef struct _%s {\n" % packet
	for variable, nBits,  _range, desc in variables:
		_, _type = getTypes(nBits, _range)				# Get type (bool, float, uint16_t, etc)
		strVar = "\t%s %s:%s;" % (_type, variable, str(nBits))
		struct += strVar.ljust(30) + "// %s\n" % desc 	# Add description
	struct += "} %s;" % packet
	return struct

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
			if i != 0 and i % 8 == 0: struct += " " # Add whitespace
			struct += "1" if i in set1 else "-" # Add '1' if bit active, '-' if inactive
		struct += " %s" % variable
		at += nBits
	return struct

def toGetters(packetName, packets):
	at = 0
	functionsString = ""

	for variable, nBits, _range, _ in packets[packetName]:
		payload = upperFirst(packetName) + "Payload" # RobotCommandPayload
		abbr = CamelCaseToAbbrev(payload)		     # rcp

		type1, type2 = getTypes(nBits, _range)

		returnValue = ""
		if type1 == "bool":
			# Find the byte index, and the bit index within that byte
			atByte, atBit = at // 8, at % 8
			returnValue = "    return ((%s->payload[%s] & %s) > 0);"
			returnValue %= (abbr, atByte, bitString(atBit, atBit+1))

		if type1 in ["uint8_t", "uint16_t", "uint32_t"]:
			
			# Collect all the bit operations, all the << and | and & and whatever needed to get the correct bits
			operations = []
			while 0 < nBits:
				# Find the byte index, and the bit index within that byte
				atByte, atBit = at // 8, at % 8
				bitsFromByte = min(nBits, 8-atBit)
				# Figure out which bits we need from the byte
				bStart, bStop = atBit, atBit + bitsFromByte
				# Generate mask, except if the entire byte is used
				mask = "" if bitsFromByte == 8 else " & %s" % bitString(bStart, bStop)
				# Figure out how much we need to shift this baby in total
				# e.g. uint16_t and 0b1110000: 16 to the left, 5 to the right => 11 to the left
				shiftRight = (8-bStop)
				shiftLeft = nBits - bitsFromByte

				operation = "((%s->payload[%d]%s)%s)"
				operation %= (abbr, atByte, mask, shift(shiftLeft, shiftRight))
				operations.append(operation)

				nBits -= bitsFromByte
				at += bitsFromByte

			# Concatenate all bit operations
			operationsStr = ' | '.join(operations)

			if type1 == type2: # immediately return the value
				returnValue = "    return %s;" % operationsStr
			elif type2 == "float": # First convert the value to float
				a, b = getConversionToFloat(nBits, _range)
				returnValue = "    %s %s = %s;\n" % (type1, variable, operationsStr)
				returnValue+= "    return (%s * %.16f) + %.16f;" % (variable, a, b)

		func = funcDeclGet % (type2, packetName, variable, payload, abbr, returnValue)
		functionsString += func
		at += nBits

	return functionsString

def toSetters(packetName, packets):
	at = 0
	functionsString = ""

	for variable, nBits, _range, _ in packets[packetName]:
		payload = upperFirst(packetName) + "Payload" # RobotCommandPayload
		abbr = CamelCaseToAbbrev(payload)		     # rcp

		type1, type2 = getTypes(nBits, _range)

		atByte = at // 8

		operations = []
		while 0 < nBits:
			# Find the byte index, and the bit index within that byte
			atByte, atBit = at // 8, at % 8
			bitsInByte = min(nBits, 8 - atBit)
			# Figure out which bits we need from the byte
			bStart, bStop = atBit, atBit + bitsInByte
			# Generate mask, except if the entire byte is used
			mask = "" if bitsInByte == 8 else " & %s" % bitString(bStart, bStop)
			# Generate inverse mask, except if the entire byte is used
			maskI= "" if bitsInByte == 8 else " & %s" % bitString(bStart, bStop, True)

			shiftLeft = 8 - bStop
			shiftRight = max(0, nBits - bitsInByte)

			inverse = ""
			if bitsInByte < 8:
				inverse = " | (%s->payload[%s]%s)" % (abbr, atByte, maskI)

			# Add shifting of variable if needed
			shiftVariable = "(%s%s%s)" % (variable, shift(0,shiftRight), shift(shiftLeft, 0))
			if shiftLeft == 0 and shiftRight == 0 : shiftVariable = variable
			# Add masking to variable if needed
			maskVariable = "(%s%s)" % (shiftVariable, mask)
			if bitsInByte == 8 : maskVariable = shiftVariable

			operation = "    %s->payload[%s] = %s%s;"
			operation %= (abbr, atByte, maskVariable, inverse)
			operations.append(operation)

			nBits -= bitsInByte
			at += bitsInByte

		operationsStr = ""
		if type2 == "float":
			a, b = getConversionToInt(nBits, _range)
			operationsStr += "    %s %s = (_%s - %.16f) * %.16f;\n" % (type1, variable, variable, b, a)

		operationsStr += '\n'.join(operations)
		_variable = "_"+variable if type2 == "float" else variable # convert 'theta' to '_theta' if a float conversion occured
		func = funcDeclSet % (packetName, variable, payload, abbr, type2, _variable, operationsStr)
		functionsString += func
		at += nBits

	return functionsString

def toDecode(packet, variables):
	payload = packet + "Payload"
	abbr1 = CamelCaseToAbbrev(packet)
	abbr2 = CamelCaseToAbbrev(payload)

	f = "static inline void decode%s(%s *%s, %s *%s){\n" % (packet, packet, abbr1, payload, abbr2)
	for variable, nBits, _, _ in variables:
		f += ("\t%s->%s"%(abbr1, variable)).ljust(25) + "= %s_get_%s(%s);\n" % (packet, variable, abbr2)
	f += "}"
	return f

def toEncode(packet, variables):
	payload = packet + "Payload"
	abbr1 = CamelCaseToAbbrev(payload)
	abbr2 = CamelCaseToAbbrev(packet)

	f = "static inline void encode%s(%s *%s, %s *%s){\n" % (packet, payload, abbr1, packet, abbr2)
	for variable, nBits, _, _ in variables:
		f += "\t%s_set_%s(%s, %s->%s);\n" % (packet, variable.ljust(20), abbr1, abbr2, variable)
	f += "}"
	return f