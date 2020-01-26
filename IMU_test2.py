# -*- coding: utf-8 -*-

import time, math, datetime
import serial, binascii, struct


# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/tty.SLAB_USBtoUART',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

while 1 :
    time.sleep(1)
    while True:
        response = ser.read()
        if response == b'\xfa':
            response = ser.read()
            if response == b'\xff':
                MID = ser.read()
                if MID == b'\x42':
                    pass
                elif MID == b'\x36':
                    data_length = ser.read()
                    data_length = binascii.hexlify(data_length)
                    data_length = data_length.decode(errors = 'ignore')
                    data_length = int(data_length, 16)
                    data = ser.read(data_length)
                    data = binascii.hexlify(data)
                    count_data = 0
                    while count_data < data_length:
                        data_ID = ser.read(2)
                        print_ID = binascii.hexlify(data_ID)
                        count_data += 2
                        if data_ID == b'\x10\x20':
                            packet_count_length = ser.read()
                            count_data += 1
                            packet_count_length = binascii.hexlify(packet_count_length)
                            packet_count_length = packet_count_length.decode(errors = 'ignore')
                            packet_count_length = int(packet_count_length, 16)
                            packet_count = ser.read(packet_count_length)
                            count_data += packet_count_length
                        if data_ID == b'\x20\x38':
                            data_size = ser.read()
                            count_data += 1
                            data_size = binascii.hexlify(data_size)
                            data_size = data_size.decode(errors = 'ignore')
                            data_size = int(data_size, 16)
                            euler_list = []
                            for i in range(math.floor(data_size/4)):
                                item = ser.read(4)
                                count_data += 4
                                item = str(binascii.hexlify(item))[2:-1]
                                item = struct.unpack('!f', bytes.fromhex(item))[0]
                                euler_list.append(item)
                            print('Euler Angles: ', euler_list)
                            print(datetime.datetime.now().time())
                        if data_ID == b'\x80\x20':
                            data_size = ser.read()
                            count_data += 1
                            data_size = binascii.hexlify(data_size)
                            data_size = data_size.decode(errors = 'ignore')
                            data_size = int(data_size, 16)
                            rate_of_turn = []
                            for i in range(math.floor(data_size/4)):
                                item = ser.read(4)
                                count_data += 4
                                item = str(binascii.hexlify(item))[2:-1]
                                item = struct.unpack('!f', bytes.fromhex(item))[0]
                                rate_of_turn.append(item)
                            print('Rate of Turn: ', rate_of_turn)
                        if data_ID == b'\x40\x20':
                            data_size = ser.read()
                            count_data += 1
                            data_size = binascii.hexlify(data_size)
                            data_size = data_size.decode(errors = 'ignore')
                            data_size = int(data_size, 16)
                            acceleration_list = []
                            for i in range(math.floor(data_size/4)):
                                item = ser.read(4)
                                count_data += 4
                                item = str(binascii.hexlify(item))[2:-1]
                                item = struct.unpack('!f', bytes.fromhex(item))[0]
                                acceleration_list.append(item)
                            print('Acceleration XYZ: ', acceleration_list)
                check_sum = ser.read()
                check_sum = binascii.hexlify(check_sum)
