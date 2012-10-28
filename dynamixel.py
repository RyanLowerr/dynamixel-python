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

# MX Control Table
MX_MODEL_NUMBER_L         = 0
MX_MODEL_NUMBER_H         = 1
MX_FIRMWARE_VERSION       = 2
MX_SERVO_ID               = 3
X_BAUD_RATE               = 4
MX_RETURN_DELAY_TIME      = 5
MX_CW_ANGLE_LIMIT_L       = 6
MX_CW_ANGLE_LIMIT_H       = 7
MX_CCW_ANGLE_LIMIT_L      = 8
MX_CCW_ANGLE_LIMIT_H      = 9
MX_LIMIT_TEMPERATURE      = 11
MX_LOW_LIMIT_VOLTAGE      = 12
MX_HIGH_LIMIT_VOLTAGE     = 13
MX_MMX_TORQUE_L           = 14
MX_MMX_TORQUE_H           = 15
MX_RETURN_LEVEL           = 16
MX_ALARM_LED              = 17
MX_ALARM_SHUTDOWN         = 18
MX_DOWN_CALIBRATION_L     = 20
MX_DOWN_CALIBRATION_H     = 21
MX_UP_CALIBRATION_L       = 22
MX_UP_CALIBRATION_H       = 23
MX_TORQUE_ENABLE          = 24
MX_LED                    = 25
MX_CW_COMPLIANCE_MARGIN   = 26
MX_CCW_COMPLIANCE_MARGIN  = 27
MX_CW_COMPLIANCE_SLOPE    = 28
MX_CCW_COMPLIANCE_SLOPE   = 29
MX_GOAL_POSITION_L        = 30
MX_GOAL_POSITION_H        = 31
MX_GOAL_SPEED_L           = 32
MX_GOAL_SPEED_H           = 33
MX_TORQUE_LIMIT_L         = 34
MX_TORQUE_LIMIT_H         = 35
MX_PRESENT_POSITION_L     = 36
MX_PRESENT_POSITION_H     = 37
MX_PRESENT_SPEED_L        = 38
MX_PRESENT_SPEED_H        = 39
MX_PRESENT_LOAD_L         = 40
MX_PRESENT_LOAD_H         = 41
MX_PRESENT_VOLTAGE        = 42
MX_PRESENT_TEMPERATURE    = 43
MX_REGISTERED_INSTRUCTION = 44
MX_MOVING                 = 46
MX_LOCK                   = 47
MX_PUNCH_L                = 48
MX_PUNCH_H                = 49

# MX Control Table
MX_MODEL_NUMBER_L         = 0
MX_MODEL_NUMBER_H         = 1
MX_FIRMWARE_VERSION       = 2
MX_SERVO_ID               = 3
MX_BAUD_RATE              = 4
MX_RETURN_DELAY_TIME      = 5
MX_CW_ANGLE_LIMIT_L       = 6
MX_CW_ANGLE_LIMIT_H       = 7
MX_CCW_ANGLE_LIMIT_L      = 8
MX_CCW_ANGLE_LIMIT_H      = 9
MX_LIMIT_TEMPERATURE      = 11
MX_LOW_LIMIT_VOLTAGE      = 12
MX_HIGH_LIMIT_VOLTAGE     = 13
MX_MMX_TORQUE_L           = 14
MX_MMX_TORQUE_H           = 15
MX_RETURN_LEVEL           = 16
MX_ALARM_LED              = 17
MX_ALARM_SHUTDOWN         = 18
MX_TORQUE_ENABLE          = 24
MX_LED                    = 25
MX_D_GAIN                 = 26
MX_I_GAIN                 = 27
MX_P_GAIN                 = 28
MX_GOAL_POSITION_L        = 30
MX_GOAL_POSITION_H        = 31
MX_GOAL_SPEED_L           = 32
MX_GOAL_SPEED_H           = 33
MX_TORQUE_LIMIT_L         = 34
MX_TORQUE_LIMIT_H         = 35
MX_PRESENT_POSITION_L     = 36
MX_PRESENT_POSITION_H     = 37
MX_PRESENT_SPEED_L        = 38
MX_PRESENT_SPEED_H        = 39
MX_PRESENT_LOAD_L         = 40
MX_PRESENT_LOAD_H         = 41
MX_PRESENT_VOLTAGE        = 42
MX_PRESENT_TEMPERATURE    = 43
MX_REGISTERED_INSTRUCTION = 44
MX_MOVING                 = 46
MX_LOCK                   = 47
MX_PUNCH_L                = 48
MX_PUNCH_H                = 49
MX_CURRENT_L              = 68
MX_CURRENT_H              = 69
MX_TORQUE_CONTROL_MODE    = 70
MX_GOAL_TORQUE_L          = 71
MX_GOAL_TORQUE_H          = 72
MX_GOAL_ACCELERATION      = 73

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
