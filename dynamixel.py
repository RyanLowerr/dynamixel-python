import serial
import time

DYNAMIXEL_ID = 2
DYNAMIXEL_LENGTH = 3
DYNAMIXEL_INSTRUCTION = 4
DYNAMIXEL_PARAMETER = 5
DYNAMIXEL_WRITE = 3
DYNAMIXEL_READ = 2
DYNAMIXEL_PING = 2
DYNAMIXEL_RESET = 2

class dynamixel:

	def __init__(self):
		self.serial = serial.Serial(UART2_PORT, UART2_BAUD)

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
			serial.write(chr(txp[i]))

	        if txp[DYNAMIXEL_INSTRUCTION] != 254:
	                if txp[DYNAMIXEL_INSTRUCTION] == DYNAMIXEL_READ: rxlength = txp[DYNAMIXEL_PARAMETER + 1] + 6
	                else: rxlength = 6

	                time.sleep(.02)
        	        for x in range(serial.inWaiting()): rxp[x] = ord(serial.read())
        	        for x in range(txlength + 1): rxp.pop(0)

	                if rxp[0] != 255 and rxp[1] != 255: return -2
        	        if rxp[rxlength - 1] != checksum(rxp): return -3
	
        	        return 1

	        return 1

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
	        rxpacket = [0]*30
		table = [0]*length

	        txpacket[DYNAMIXEL_ID] = id
	        txpacket[DYNAMIXEL_LENGTH] = 4
	        txpacket[DYNAMIXEL_INSTRUCTION] = DYNAMIXEL_READ
	        txpacket[DYNAMIXEL_PARAMETER] = start_addr
	        txpacket[DYNAMIXEL_PARAMETER + 1] = length

		result = self.txrx(txpacket, rxpacket) 

		if(result == DYNAMIXEL_SUCCESS):
			for i in range(length):
				table[i] = rxpacket[DYNAMIXEL_PARAMETER + 1]

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