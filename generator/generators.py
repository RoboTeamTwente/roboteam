import math



funcDeclGet = """static inline %s %s_get%s(%s *%s){
	return %s;
}"""

funcDeclSet = """static inline void %s_set%s(%s *%s, %s %s){
%s
}"""


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

def toHeaderGuard(packet):
	return "_%s_H" % CamelCaseToUpper(packet)

def toStructPayload(packet):
	struct = "typedef struct _%sPayload {\n\tuint8_t payload[PACKET_SIZE%s];\n} %sPayload;"
	struct %= (packet, CamelCaseToUpper(packet), packet)
	return struct

def toStruct(packet, variables):
	struct = "typedef struct _%s {\n" % packet
	for _type, variable, nBits, desc in variables:
		strVar = "\t%s %s;" % (_type, variable)
		struct += strVar.ljust(30) + "// %s\n" % desc
	struct += "} %s;" % packet
	return struct

def toStructure(packet, variables):
	nBytes = math.ceil(sum([nBits for (_, _, nBits, _) in variables]) / 8)

	struct = ' '.join(["[%s--]" % (("%d"%i).rjust(4,"-")) for i in range(nBytes)])
	at = 0
	for _type, variable, nBits, desc in variables:
		set1 = list(range(at, at+nBits))
		struct += "\n"	
		for i in range(nBytes * 8):
			if i != 0 and i % 8 == 0: struct += " "
			struct += "1" if i in set1 else "-"
		struct += " %s" % variable
		at += nBits
	return struct

def toGetter(_type, variable, nBits, packet, at):
	payload = upperFirst(packet) + "Payload" # RobotCommandPayload
	var = CamelCaseToAbbrev(payload)		 # rcp

	returnValue = ""

	if _type == "bool":
		# Find the byte index, and the bit index within that byte
		atByte, atBit = at // 8, at % 8
		returnValue = "((%s->payload[%s] & %s) > 0)"
		returnValue %= (var, atByte, bitString(atBit, atBit+1))

	if _type in ["uint8_t", "uint16_t"]:
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
			operation %= (var, atByte, mask, shift(shiftLeft, shiftRight))
			operations.append(operation)

			nBits -= bitsFromByte
			at += bitsFromByte

		returnValue = ' | '.join(operations)

	func = funcDeclGet % (_type, packet, upperFirst(variable), payload, var, returnValue)
	return func

def toSetter(_type, variable, nBits, packet, at):
	payload = upperFirst(packet) + "Payload" # RobotCommandPayload
	abbr = CamelCaseToAbbrev(payload)		 # rcp

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

		operation = "\t%s->payload[%s] = %s%s;"
		operation %= (abbr, atByte, maskVariable, inverse)
		operations.append(operation)

		nBits -= bitsInByte
		at += bitsInByte

	value = '\n'.join(operations)

	func = funcDeclSet % \
	(packet, upperFirst(variable), payload, abbr, _type, variable, value)
	return func

def toDecode(packet, variables):
	payload = packet + "Payload"
	abbr1 = CamelCaseToAbbrev(packet)
	abbr2 = CamelCaseToAbbrev(payload)

	f = "static inline void decode%s(%s *%s, %s *%s){\n" % (packet, packet, abbr1, payload, abbr2)
	for _type, variable, nBits, _ in variables:
		f += ("\t%s->%s"%(abbr1, variable)).ljust(25) + "= %s_get%s(%s);\n" % (packet, upperFirst(variable), abbr2)
	f += "}"
	return f

def toEncode(packet, variables):
	payload = packet + "Payload"
	abbr1 = CamelCaseToAbbrev(payload)
	abbr2 = CamelCaseToAbbrev(packet)

	f = "static inline void encode%s(%s *%s, %s *%s){\n" % (packet, payload, abbr1, packet, abbr2)
	for _type, variable, nBits, _ in variables:
		f += "\t%s_set%s(%s, %s->%s);\n" % (packet, upperFirst(variable), abbr1, abbr2, variable)
	f += "}"
	return f