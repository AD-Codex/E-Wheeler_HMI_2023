#!/usr/bin/python3
from bluepy import btle
import math
import json


EXTENDED_RECORD = 1
CELL_DATA = 2
INFO_RECORD = 3

getInfo = b'\xaa\x55\x90\xeb\x97\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x11'
getCellInfo = b'\xaa\x55\x90\xeb\x96\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10'

name = 'wtfsection'
model = 'JK-BD6A20S10P'
mac = 'C8:47:BC:E8:3A:B4'
command = 'command'
tag = 'Powerwall_1'
format = 'influx2'

out=dict()
c_high=0.00
c_low=10.00
c_diff=0.00

class jkBmsDelegate(btle.DefaultDelegate):
    '''
    BLE delegate to deal with notifications (information) from the JKBMS device
    '''

    def __init__(self, jkbms):
        btle.DefaultDelegate.__init__(self)
        # extra initialisation here
        self.jkbms = jkbms
        #print('Delegate {}'.format(str(jkbms)))
        self.notificationData = bytearray()

    def recordIsComplete(self):
        '''
        '''
        # print('Notification Data {}'.format(self.notificationData))
        # check for 'ack' record
        if self.notificationData.startswith(bytes.fromhex('aa5590eb')):
            #print('notificationData has ACK')
            self.notificationData = bytearray()
            return False  # strictly record is complete, but we dont process this
        # check record starts with 'SOR'
        SOR = bytes.fromhex('55aaeb90')
        if not self.notificationData.startswith(SOR):
            print('No SOR found in notificationData')
            self.notificationData = bytearray()
            return False
        # check that length one of the valid lengths (300, 320)
        if len(self.notificationData) == 300 or len(self.notificationData) == 320:
            # check the crc/checksum is correct for the record data
            crc = ord(self.notificationData[-1:])
            calcCrc = crc8(self.notificationData[:-1])
            # print (crc, calcCrc)
            if crc == calcCrc:
                return True
        return False

    def processInfoRecord(self, record):
        #print('Processing info record')
        del record[0:5]
        counter = record.pop(0)
        #print('Record number: {}'.format(counter))
        vendorID = bytearray()
        hardwareVersion = bytearray()
        softwareVersion = bytearray()
        uptime = 0
        powerUpTimes = 0
        deviceName = bytearray()
        passCode = bytearray()
        # start at byte 7, go till 0x00 for device model
        while len(record) > 0:
            _int = record.pop(0)
            # print (_int)
            if _int == 0x00:
                break
            else:
                vendorID += bytes(_int.to_bytes(1, byteorder='big'))
        # consume remaining null bytes
        _int = record.pop(0)
        while _int == 0x00:
            _int = record.pop(0)
        # process hardware version
        hardwareVersion += bytes(_int.to_bytes(1, byteorder='big'))
        while len(record) > 0:
            _int = record.pop(0)
            # print (_int)
            if _int == 0x00:
                break
            else:
                hardwareVersion += bytes(_int.to_bytes(1, byteorder='big'))
        # consume remaining null bytes
        _int = record.pop(0)
        while _int == 0x00:
            _int = record.pop(0)
        # process software version
        softwareVersion += bytes(_int.to_bytes(1, byteorder='big'))
        while len(record) > 0:
            _int = record.pop(0)
            # print (_int)
            if _int == 0x00:
                break
            else:
                softwareVersion += bytes(_int.to_bytes(1, byteorder='big'))
        # consume remaining null bytes
        _int = record.pop(0)
        while _int == 0x00:
            _int = record.pop(0)
        # process uptime version
        upTimePos = 0
        uptime = _int * 256**upTimePos
        while len(record) > 0:
            _int = record.pop(0)
            upTimePos += 1
            # print (_int)
            if _int == 0x00:
                break
            else:
                uptime += _int * 256**upTimePos
        # consume remaining null bytes
        _int = record.pop(0)
        while _int == 0x00:
            _int = record.pop(0)
        # power up times
        powerUpTimes = _int
        # consume remaining null bytes
        _int = record.pop(0)
        while _int == 0x00:
            _int = record.pop(0)
        # device name
        deviceName += bytes(_int.to_bytes(1, byteorder='big'))
        while len(record) > 0:
            _int = record.pop(0)
            # print (_int)
            if _int == 0x00:
                break
            else:
                deviceName += bytes(_int.to_bytes(1, byteorder='big'))
        # consume remaining null bytes
        _int = record.pop(0)
        while _int == 0x00:
            _int = record.pop(0)
        # Passcode
        passCode += bytes(_int.to_bytes(1, byteorder='big'))
        while len(record) > 0:
            _int = record.pop(0)
            # print (_int)
            if _int == 0x00:
                break
            else:
                passCode += bytes(_int.to_bytes(1, byteorder='big'))

        #print('VendorID: {}'.format(vendorID.decode('utf-8')))
        #publish({'VendorID': vendorID.decode('utf-8')}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        #print('Device Name: {}'.format(deviceName.decode('utf-8')))
        #publish({'DeviceName': deviceName.decode('utf-8')}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        #print('Pass Code: {}'.format(passCode.decode('utf-8')))
        # #publish({'PassCode': passCode.decode('utf-8')}, format=self.jkbms.format, broker=self.jkbms.mqttBroker)
        #print('Hardware Version: {}'.format(hardwareVersion.decode('utf-8')))
        #publish({'HardwareVersion': hardwareVersion.decode('utf-8')}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        #print('Software Version: {}'.format(softwareVersion.decode('utf-8')))
        #publish({'SoftwareVersion': softwareVersion.decode('utf-8')}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        daysFloat = uptime / (60 * 60 * 24)
        days = math.trunc(daysFloat)
        hoursFloat = (daysFloat - days) * 24
        hours = math.trunc(hoursFloat)
        minutesFloat = (hoursFloat - hours) * 60
        minutes = math.trunc(minutesFloat)
        secondsFloat = (minutesFloat - minutes) * 60
        seconds = math.trunc(secondsFloat)
        #print('Uptime: {}D{}H{}M{}S'.format(days, hours, minutes, seconds))
        #publish({'Uptime': '{}D{}H{}M{}S'.format(days, hours, minutes, seconds)}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        #print('Power Up Times: {}'.format(powerUpTimes))
        #publish({'Power Up Times: {}'.format(powerUpTimes)}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)

    def processExtendedRecord(self, record):
        #print('Processing extended record')
        del record[0:5]
        counter = record.pop(0)
        #print('Record number: {}'.format(counter))

    def processCellDataRecord(self, record):
        #print('Processing cell data record')
        #print('Record length {}'.format(len(record)))
        del record[0:5]
        counter = record.pop(0)
        #print('Record number: {}'.format(counter))
        # Process cell voltages
        volts = []
        size = 4
        number = 24
        c_high=0
        c_low=10
        c_diff=0
        for i in range(0, number):
            volts.append(record[0:size])
            del record[0:size]
        #print('Volts: {}'.format(volts))
        _totalvolt = 0
        for cell, volt in enumerate(volts):
            _volt = float(decodeHex(volt))
            out['B{:d}'.format(cell+1)]=round(_volt,4)
            #print('Cell: {:02d}, Volts: {:.4f}'.format(cell + 1, _volt))
            #publish({'VoltageCell{:02d}'.format(cell + 1): _volt}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
            _totalvolt += _volt
            if c_high < _volt:
                c_high= _volt
            if c_low > _volt and _volt != 0:
                c_low=_volt
        #publish({'VoltageTotal': _totalvolt}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        # Process cell wire resistances
        # print (record)
        #print('Processing wire resistances')
        out["Total"]= round(_totalvolt,4)
        out["Cell_High"]= round(c_high,4)
        out["Cell_Low"]= round(c_low,4)
        out["Cell_Diff"]= round(c_high-c_low,4)
        resistances = []
        size = 4
        number = 25
        for i in range(0, number):
            resistances.append(record[0:size])
            del record[0:size]
        for cell, resistance in enumerate(resistances):
            out['R{:d}'.format(cell+1)]=round(decodeHex(resistance),4)
            #print('Cell: {:02d}, Resistance: {:.4f}'.format(cell, decodeHex(resistance)))
            #publish({'ResistanceCell{:02d}'.format(cell): float(decodeHex(resistance))}, format=self.jkbms.format, broker=self.jkbms.mqttBroker, tag=self.jkbms.tag)
        # print (record)

    def processRecord(self, record):
        recordType = record[4]
        # counter = record[5]
        if recordType == INFO_RECORD:
            self.processInfoRecord(record)
        elif recordType == EXTENDED_RECORD:
            self.processExtendedRecord(record)
        elif recordType == CELL_DATA:
            self.processCellDataRecord(record)
        else:
            print('Unknown record type')

    def handleNotification(self, handle, data):
        # handle is the handle of the characteristic / descriptor that posted the notification
        # data is the data in this notification - may take multiple notifications to get all of a message
        #print('From handle: {:#04x} Got {} bytes of data'.format(handle, len(data)))
        self.notificationData += bytearray(data)
        if self.recordIsComplete():
            record = self.notificationData
            self.notificationData = bytearray()
            self.processRecord(record)


class jkBMS:
    """
    JK BMS Command Library
    - represents a JK BMS
    """

#    def __str__(self):
        #return 'JKBMS instance --- name: {}, model: {}, mac: {}, command: {}, tag: {}, format: {}, records: {}, maxConnectionAttempts: {}, mqttBroker: {}'.format(self.name, self.model, self.mac, self.command, self.tag, self.format, self.records, self.maxConnectionAttempts, self.mqttBroker)
#        return out

    def __init__(self, name, model, mac, command, tag, format, records=1, maxConnectionAttempts=3, mqttBroker=None):
        '''
        '''
        self.name = name
        self.model = model
        self.mac = mac
        self.command = command
        self.tag = tag
        self.format = format
        try:
            self.records = int(records)
        except Exception:
            self.records = 1
        self.maxConnectionAttempts = maxConnectionAttempts
        self.mqttBroker = mqttBroker
        self.device = btle.Peripheral(None)
        #print('Config data - name: {}, model: {}, mac: {}, command: {}, tag: {}, format: {}'.format(self.name, self.model, self.mac, self.command, self.tag, self.format))
        #print('Additional config - records: {}, maxConnectionAttempts: {}, mqttBroker: {}'.format(self.records, self.maxConnectionAttempts, self.mqttBroker))
        #print('jkBMS Logging level: {}'.format(log.level))

    def connect(self):
        # Intialise BLE device
        self.device = btle.Peripheral(None)
        self.device.withDelegate(jkBmsDelegate(self))
        # Connect to BLE Device
        connected = False
        attempts = 0
        print('Attempting to connect to {}'.format(self.name))
        while not connected:
            attempts += 1
            if attempts > self.maxConnectionAttempts:
                print('Cannot connect to {} with mac {} - exceeded {} attempts'.format(self.name, self.mac, attempts - 1))
                return connected
            try:
                self.device.connect(self.mac)
                connected = True
                print("connected")
            except Exception:
                continue
        return connected

    def getBLEData(self):
        # Get the device name
        serviceId = self.device.getServiceByUUID(btle.AssignedNumbers.genericAccess)
        deviceName = serviceId.getCharacteristics(btle.AssignedNumbers.deviceName)[0]
        #print('Connected to {}'.format(deviceName.read()))

        # Connect to the notify service
        serviceNotifyUuid = 'ffe0'
        serviceNotify = self.device.getServiceByUUID(serviceNotifyUuid)

        # Get the handles that we need to talk to
        # Read
        characteristicReadUuid = 'ffe3'
        characteristicRead = serviceNotify.getCharacteristics(characteristicReadUuid)[0]
        handleRead = characteristicRead.getHandle()
        #print('Read characteristic: {}, handle {:x}'.format(characteristicRead, handleRead))

        # ## TODO sort below
        # Need to dynamically find this handle....
        #print('Enable 0x0b handle', self.device.writeCharacteristic(0x0b, b'\x01\x00'))
        self.device.writeCharacteristic(0x0b, b'\x01\x00')
        #print('Enable read handle', self.device.writeCharacteristic(handleRead, b'\x01\x00'))
        self.device.writeCharacteristic(handleRead, b'\x01\x00')
        #print('Write getInfo to read handle', self.device.writeCharacteristic(handleRead, getInfo))
        self.device.writeCharacteristic(handleRead, getInfo)
        secs = 0
        while True:
            if self.device.waitForNotifications(1.0):
                continue
            secs += 1
            if secs > 5:
                break

        #print('Write getCellInfo to read handle', self.device.writeCharacteristic(handleRead, getCellInfo))
        self.device.writeCharacteristic(handleRead, getCellInfo)
        loops = 0
        recordsToGrab = self.records
        #print('Grabbing {} records (after inital response)'.format(recordsToGrab))

        while True:
            loops += 1
            if loops > recordsToGrab * 15 + 16:
                #print('Got {} records'.format(recordsToGrab))
                break
            if self.device.waitForNotifications(1.0):
                continue

    def disconnect(self):
        #print('Disconnecting...')
        self.device.disconnect()

		
def crc8(byteData):
    '''
    Generate 8 bit CRC of supplied string
    '''
    CRC = 0
    # for j in range(0, len(str),2):
    for b in byteData:
        # char = int(str[j:j+2], 16)
        # print(b)
        CRC = CRC + b
        CRC &= 0xff
    return CRC


def decodeHex(hexToDecode):
    '''
    Code a 4 byte hexString to volts as per jkbms approach (blackbox determined)
    '''
    # hexString = bytes.fromhex(hexToDecode)
    hexString = hexToDecode
    # print('hexString: {}'.format(hexString))

    answer = 0.0

    # Make sure supplied String is long enough
    if len(hexString) != 4:
        print('Hex encoded value must be 4 bytes long. Was {} length'.format(len(hexString)))
        return 0

    # Process most significant byte (position 3)
    byte1 = hexString[3]
    if byte1 == 0x0:
        return 0
    byte1Low = byte1 - 0x40
    answer = (2**(byte1Low * 2)) * 2
    #print('After position 3: {}'.format(answer))
    step1 = answer / 8.0
    step2 = answer / 128.0
    step3 = answer / 2048.0
    step4 = answer / 32768.0
    step5 = answer / 524288.0
    step6 = answer / 8388608.0

    # position 2
    byte2 = hexString[2]
    byte2High = byte2 >> 4
    byte2Low = byte2 & 0xf
    if byte2High & 8:
        answer += ((byte2High - 8) * step1 * 2) + (8 * step1) + (byte2Low * step2)
    else:
        answer += (byte2High * step1) + (byte2Low * step2)
    #print('After position 2: {}'.format(answer))
    # position 1
    byte3 = hexString[1]
    byte3High = byte3 >> 4
    byte3Low = byte3 & 0xf
    answer += (byte3High * step3) + (byte3Low * step4)
    #print('After position 1: {}'.format(answer))
    # position 0
    byte4 = hexString[0]
    byte4High = byte4 >> 4
    byte4Low = byte4 & 0xf
    answer += (byte4High * step5) + (byte4Low * step6)
    #print('After position 0: {}'.format(answer))

    #print('hexString: {}'.format(hexString))
    #print('hex(byte1): {}'.format(hex(byte1)))
    #print('byte1Low: {}'.format(byte1Low))
    # print ('byte2', byte2)
    #print('hex(byte2): {}'.format(hex(byte2)))
    #print('byte2High: {}'.format(byte2High))
    #print('byte2Low: {}'.format(byte2Low))
    # print ('byte3', byte3)
    #print('hex(byte3): {}'.format(hex(byte3)))
    #print('byte3High: {}'.format(byte3High))
    #print('byte3Low: {}'.format(byte3Low))
    # print ('byte4', byte4)
    #print('hex(byte4): {}'.format(hex(byte4)))
    #print('byte4High: {}'.format(byte4High))
    #print('byte4Low: {}'.format(byte4Low))

    #print('step1: {}'.format(step1))
    #print('step2: {}'.format(step2))
    #print('step3: {}'.format(step3))
    #print('step4: {}'.format(step4))
    #print('step5: {}'.format(step5))
    #print('step6: {}'.format(step6))

    #print('Hex {} decoded to {}'.format(hexString, answer))

    return answer





jk = jkBMS(name=name, model=model, mac=mac, command=command, tag=tag, format=format, records=1, maxConnectionAttempts=3)
print(str(jk))
jk.connect()
# if jk.connect():
#     jk.getBLEData()
jk.disconnect()
# else:
#     print('Failed to connect to {} {}'.format(name, mac))
print(json.dumps(out))
#print('wtf')
