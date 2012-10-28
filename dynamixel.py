import serial
import time

DYNAMIXEL_ID	          = 2
DYNAMIXEL_LENGTH          = 3
DYNAMIXEL_INSTRUCTION     = 4
DYNAMIXEL_ERROR	          = 4
DYNAMIXEL_PARAMETER       = 5

DYNAMIXEL_BROADCAST_ID    = 254

DYNAMIXEL_PING	          = 1
DYNAMIXEL_READ	          = 2
DYNAMIXEL_WRITE	          = 3
DYNAMIXEL_REG_WRITE       = 4
DYNAMIXEL_ACTION          = 5
DYNAMIXEL_RESET	          = 6
DYNAMIXEL_SYNC_WRITE      = 131

DYNAMIXEL_SUCCESS         = 1
DYNAMIXEL_RX_CORRUPT      = 2
DYNAMIXEL_RX_TIMEOUT      = 3
DYNAMIXEL_TX_FAIL         = 4
DYNAMIXEL_TX_TIMEOUT      = 5

# AX Control Table
AX_MODEL_NUMBER_L         = 0
AX_MODEL_NUMBER_H         = 1
AX_VERSION                = 2
AX_SERVO_ID               = 3
X_BAUD_RATE               = 4
AX_RETURN_DELAY_TIME      = 5
AX_CW_ANGLE_LIMIT_L       = 6
AX_CW_ANGLE_LIMIT_H       = 7
AX_CCW_ANGLE_LIMIT_L      = 8
AX_CCW_ANGLE_LIMIT_H      = 9
AX_LIMIT_TEMPERATURE      = 11
AX_LOW_LIMIT_VOLTAGE      = 12
AX_HIGH_LIMIT_VOLTAGE     = 13
AX_MAX_TORQUE_L           = 14
AX_MAX_TORQUE_H           = 15
AX_RETURN_LEVEL           = 16
AX_ALARM_LED              = 17
AX_ALARM_SHUTDOWN         = 18
AX_DOWN_CALIBRATION_L     = 20
AX_DOWN_CALIBRATION_H     = 21
AX_UP_CALIBRATION_L       = 22
AX_UP_CALIBRATION_H       = 23
AX_TORQUE_ENABLE          = 24
AX_LED                    = 25
AX_CW_COMPLIANCE_MARGIN   = 26
AX_CCW_COMPLIANCE_MARGIN  = 27
AX_CW_COMPLIANCE_SLOPE    = 28
AX_CCW_COMPLIANCE_SLOPE   = 29
AX_GOAL_POSITION_L        = 30
AX_GOAL_POSITION_H        = 31
AX_GOAL_SPEED_L           = 32
AX_GOAL_SPEED_H           = 33
AX_TORQUE_LIMIT_L         = 34
AX_TORQUE_LIMIT_H         = 35
AX_PRESENT_POSITION_L     = 36
AX_PRESENT_POSITION_H     = 37
AX_PRESENT_SPEED_L        = 38
AX_PRESENT_SPEED_H        = 39
AX_PRESENT_LOAD_L         = 40
AX_PRESENT_LOAD_H         = 41
AX_PRESENT_VOLTAGE        = 42
AX_PRESENT_TEMPERATURE    = 43
AX_REGISTERED_INSTRUCTION = 44
AX_MOVING                 = 46
AX_LOCK                   = 47
AX_PUNCH_L                = 48
AX_PUNCH_H                = 49

# MX Control Table

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
