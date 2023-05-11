import serial
import time

# Checksum 4bytes, bytes 1-2 0000 not used, bytes 3-4 cumulative total 
def crc(byteData):
    CRC = 0
    for b in byteData:
        print(("{0:02x}".format(b)))
        CRC += b
    crc_byte4 = CRC & 0xFF
    crc_byte3 = (CRC >> 8) & 0xFF
    print(("{0:02x}".format(crc_byte3)))
    print(("{0:02x}".format(crc_byte4)))
    return [crc_byte3, crc_byte4]

# 4.2.4 COMMAND codes
command_ACTIVATE = b'\x01'
command_WRITE = b'\x02'
command_READ = b'\x03'
command_SEND_PASSWORD = b'\x05'
command_READ_ALL_DATA = b'\x06'

# FRAME SOURCE 0 BMS Data box 1 Bluetooth 2 GPS 3 PC Host Computer
source_BMS_DATA_BOX = b'\x00'
source_BLUETOOTH = b'\x01'
source_HOST_PC = b'\x03'

# Transmission type
tx_type_READ_DATA = b'\x00'
tx_type_REPLY_FRAME = b'\x01'
tx_type_WRITE_DATA = b'\x02'

# 4.2 Frame Format
frame_STX = b'\x4E\x57'
frame_LENGTH = bytearray(2)
frame_BMS_ID = bytearray(4)
frame_Command = bytearray(1)
frame_SOURCE = bytearray(1)
frame_TX_Type = bytearray(1)
frame_INFO_READ = bytearray(1)
frame_REC_NUM = bytearray(4)
frame_END_FLAG = b'\x68'
frame_CRC_HIGH = b'\x00\x00'
frame_CRC_LOW = bytearray(2)

# Initialise read request
# For READ req. LENGTH = 19, 0x0013
frame_LENGTH = b'\x00\x13'
frame_BMS_ID = b'\x00\x00\x00\x00'

frame_Command = command_READ_ALL_DATA
#frame_Command = command_ACTIVATE

#frame_SOURCE = source_BMS_DATA_BOX
frame_SOURCE = source_HOST_PC

frame_TX_Type = tx_type_READ_DATA

frame_INFO_READ = b'\x00'

frame_REC_NUM = b'\x00\x00\x00\x00'

# 4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29

# voltage
# 4E 57 00 13 00 00 00 00 03 03 00 83 00 00 00 00 68 00 00 01 A9


# Make request frame
request_FRAME = bytearray()
request_FRAME[0:2] = frame_STX
request_FRAME[2:2] = frame_LENGTH
request_FRAME[4:4] = frame_BMS_ID
# request_FRAME[8:1] = frame_Command
request_FRAME[8:1] = b'\x03'
request_FRAME[9:1] = frame_SOURCE
request_FRAME[10:1] = frame_TX_Type
# request_FRAME[11:1] = frame_INFO_READ
request_FRAME[11:1] = b'\x83'
request_FRAME[12:4] = frame_REC_NUM
request_FRAME[16:1] = frame_END_FLAG

crc_byte3, crc_byte4 = crc(request_FRAME[0:17])
frame_CRC_LOW[0] = crc_byte3
frame_CRC_LOW[1] = crc_byte4

request_FRAME[17:2] = frame_CRC_HIGH
# request_FRAME[17:2] = b'\x00\x00'
request_FRAME[19:2] = frame_CRC_LOW
# request_FRAME[19:2] = b'\x01\xA9'

port = "/dev/ttyUSB6"
baud = 115200

# request_FRAME = b'\x4E\x57\x00\x13\x00\x00\x00\x00\x06\x03\x00\x00\x00\x00\x00\x00\x68\x00\x00\x01\x29'
print (f"sending command: {request_FRAME.hex()}")
########################

with serial.serial_for_url(port, baud) as s:
    s.timeout = 1
    s.write_timeout = 1
    s.flushInput()
    s.flushOutput()
    print (f"sending command: {request_FRAME.hex()}")
    bytes_written = s.write(request_FRAME)
    print (f"wrote {bytes_written} bytes")
    # response_line = s.readline()
    # time.sleep(1)
    # print (f"Got response: {response_line}")
    for _ in range(5):
        response_line = s.readline()
        time.sleep(1)
        print (f"Got response: {response_line}")