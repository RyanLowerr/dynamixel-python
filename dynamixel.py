import serial
import time

DYNAMIXEL_ID	       = 2
DYNAMIXEL_LENGTH       = 3
DYNAMIXEL_INSTRUCTION  = 4
DYNAMIXEL_ERROR	       = 4
DYNAMIXEL_PARAMETER    = 5

DYNAMIXEL_BROADCAST_ID = 254

DYNAMIXEL_PING	       = 1
DYNAMIXEL_READ	       = 2
DYNAMIXEL_WRITE	       = 3
DYNAMIXEL_REG_WRITE    = 4
DYNAMIXEL_ACTION       = 5
DYNAMIXEL_RESET	       = 6
DYNAMIXEL_SYNC_WRITE   = 131

DYNAMIXEL_SUCCESS      = 1
DYNAMIXEL_RX_CORRUPT   = 2
DYNAMIXEL_RX_TIMEOUT   = 3
DYNAMIXEL_TX_FAIL      = 4
DYNAMIXEL_TX_TIMEOUT   = 5

class dynamixel:

	def __init__(self, port, baud):
		self.serial = serial.Serial(port, baud)

	def checksum(self, packet):
		check = 0
		for i in range(DYNAMIXEL_ID, (packet[DYNAMIXEL_LENGTH] + 3)):
			check += packet[i]
		return 255 - (check % 256)

	def txrx(self, txp, rxp):
		txlength = txp[DYNAMIXEL_LENGTH] + 4
		rxlength = 0

		txp[0] = 0xff
		txp[1] = 0xff
		txp[txlength - 1] = self.checksum(txp)

		for i in range(txlength):
			self.serial.write(chr(txp[i]))

		if txp[DYNAMIXEL_INSTRUCTION] != 254:
			if txp[DYNAMIXEL_INSTRUCTION] == DYNAMIXEL_READ: 
				rxlength = txp[DYNAMIXEL_PARAMETER + 1] + 6
			else: 
				rxlength = 6

			time.sleep(.02)
			
			for x in range(self.serial.inWaiting()): 
				rxp[x] = ord(self.serial.read())

			if rxp[0] != 255 and rxp[1] != 255: 
				return DYNAMIXEL_RX_CORRUPT
			
			if rxp[rxlength - 1] != self.checksum(rxp): 
				return DYNAMIXEL_RX_CORRUPT

		return DYNAMIXEL_SUCCESS

	def ping(self, id):
		txpacket = [0]*8
		rxpacket = [0]*30

		txpacket[DYNAMIXEL_ID] = id
	   	txpacket[DYNAMIXEL_LENGTH] = 2
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_PING

		return self.txrx(txpacket, rxpacket)

	def reset(self, id):
		txpacket = [0]*8
		rxpacket = [0]*30
		
		txpacket[DYNAMIXEL_ID] = id
		txpacket[DYNAMIXEL_LENGTH] = 2
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_RESET
		
		return self.txrx(txpacket, rxpacket)

	def readbyte(self, id, address):
		txpacket = [0]*8
		rxpacket = [0]*30

		txpacket[DYNAMIXEL_ID] = id
		txpacket[DYNAMIXEL_LENGTH] = 4
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_READ
		txpacket[DYNAMIXEL_PARAMETER] = address
		txpacket[DYNAMIXEL_PARAMETER + 1] = 1

		result = self.txrx(txpacket, rxpacket)
		value = rxpacket[DYNAMIXEL_PARAMETER]
		return result, value

	def readword(self, id, address):
		txpacket = [0]*8
		rxpacket = [0]*30

		txpacket[DYNAMIXEL_ID] = id
		txpacket[DYNAMIXEL_LENGTH] = 4
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_READ
		txpacket[DYNAMIXEL_PARAMETER] = address
		txpacket[DYNAMIXEL_PARAMETER + 1] = 2

		result = self.txrx(txpacket, rxpacket)
		value = ((rxpacket[DYNAMIXEL_PARAMETER + 1] << 8) + rxpacket[DYNAMIXEL_PARAMETER])
		return result, value

	def readtable(self, id, start_addr, end_addr):
		length = end_addr - start_addr + 1	

		txpacket = [0]*8
		rxpacket = [0]*(length + 10)
		table = [0]*length

		txpacket[DYNAMIXEL_ID] = id
		txpacket[DYNAMIXEL_LENGTH] = 4
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_READ
		txpacket[DYNAMIXEL_PARAMETER] = start_addr
		txpacket[DYNAMIXEL_PARAMETER + 1] = length

		result = self.txrx(txpacket, rxpacket)

		if(result == DYNAMIXEL_SUCCESS):
			for i in range(length):
				table[i] = rxpacket[DYNAMIXEL_PARAMETER + i]

		return result, table

	def writebyte(self, id, address, value):
		txpacket = [0]*8
		rxpacket = [0]*30

		txpacket[DYNAMIXEL_ID] = id
		txpacket[DYNAMIXEL_LENGTH] = 4
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_WRITE
		txpacket[DYNAMIXEL_PARAMETER] = address
		txpacket[DYNAMIXEL_PARAMETER + 1] = value
		
		return self.txrx(txpacket, rxpacket)

	def writeword(self, id, address, value):
		txpacket = [0]*9
		rxpacket = [0]*30

		txpacket[DYNAMIXEL_ID] = id
		txpacket[DYNAMIXEL_LENGTH] = 5
		txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_WRITE
		txpacket[DYNAMIXEL_PARAMETER] = address
		txpacket[DYNAMIXEL_PARAMETER + 1] = value & 0xff
		txpacket[DYNAMIXEL_PARAMETER + 2] = (value & 0xff00) >> 8

		return self.txrx(txpacket, rxpacket)

dynamixel = dynamixel("/dev/ttyUSB0", 1000000)
result, table = dynamixel.readtable(1, 0, 49)

if result == DYNAMIXEL_SUCCESS:
	print " MODEL NUMBER            " + str(table[0] + (table[1] << 8))
	print " VERSION                 " + str(table[2])
	print " ID                      " + str(table[3])
	print " BAUD RATE               " + str(table[4])
	print " RETURN DELAY TIME       " + str(table[5])
	print " CW ANGLE LIMIT          " + str(table[6] + (table[7] << 8))
	print " CCW ANGLE LIMIT         " + str(table[8] + (table[9] << 8))
	print " LIMIT TEMPERATURE       " + str(table[11])
	print " LOW LIMIT VOLTAGE       " + str(table[12])
	print " HIGH LIMIT VOLTAGE      " + str(table[13])
	print " MAX TORQUE              " + str(table[14] + (table[15] << 8))
	print " RETURN LEVEL            " + str(table[16])
	print " ALARM LED               " + str(table[17])
	print " ALARM SHUTDOWN          " + str(table[18])
	print " DOWN CALIBRATION        " + str(table[20] + (table[21] << 8))
	print " UP CALIBRATION          " + str(table[22] + (table[23] << 8))
	print " TORQUE ENABLE           " + str(table[24])
	print " LED                     " + str(table[25])
	print " CW COMPLIANCE MARGIN    " + str(table[26])
	print " CCW COMPLIANCE MARGIN   " + str(table[27])
	print " CW COMPLIANCE SLOPE     " + str(table[28])
	print " CCW COMPLIANCE SLOPE    " + str(table[29])
	print " GOAL POSITION           " + str(table[30] + (table[31] << 8))
	print " GOAL SPEED              " + str(table[32] + (table[33] << 8))
	print " TORQUE LIMIT            " + str(table[34] + (table[35] << 8))
	print " PRESENT POSITION        " + str(table[36] + (table[37] << 8))
	print " PRESENT SPEED           " + str(table[38] + (table[39] << 8))
	print " PRESENT LOAD            " + str(table[40] + (table[41] << 8))
	print " PRESENT VOLTAGE         " + str(table[42])
	print " PRESENT TEMPERATURE     " + str(table[43])
	print " REGISTERED INSTRUCTION  " + str(table[44])
	print " MOVING                  " + str(table[46])
	print " LOCK                    " + str(table[47])
	print " PUNCH                   " + str(table[48] + (table[49] << 8))
