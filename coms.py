from serial import Serial
import struct
import copy


""" Match the names in the arudino sketch """
MSGT_NONE = 0
MSGT_MOT_TARE = 1           # zero the step counter
MSGT_MOT_HOME = 2           # move to towards tare location
MSGT_START_SAMPLING = 3     # start motor
MSGT_STOP_SAMPLING = 4      # stop motor
MSGT_SET_SPEED = 5          # motor speed, ie elongation rate
MSGT_SAMPLE = 6             # outgoing data packet


MSG_HEADER = [b't', b't']

class outgoingStruct:
    _type = None

class setMotSpeedStruct:
    _type = MSGT_SET_SPEED
    mot_speed = 0
    mot_dir = 0

class sampleStruct:
    _type = MSGT_SAMPLE
    load = 0
    steps = 0
    displacement = 0

    def __str__(self):
        return "Load={}, Steps={}, Displacement={}".format(self.load, self.steps, self.displacement)


def start_serial(port="COM3", baudrate=115200):
    arduino = Serial(
        port=port,
        baudrate=baudrate,
    )
    return arduino


""" Arduino uses little endian """
def read_unit8_t(data):
    [data.append(b'0') for i in range(1-len(data))]
    return struct.unpack('<b', b''.join(data))[0]

def read_unit16_t(data):
    [data.append(b'0') for i in range(2-len(data))]
    return struct.unpack('<h', b''.join(data))[0]

def read_unit32_t(data):
    [data.append(b'0') for i in range(4-len(data))]
    return struct.unpack('<l', b''.join(data))[0]

def write_uint8_t(data):
    """ data is type uint8_t """
    if data < 0 or data > 255:
        print("write_uint8_t value error: {}".format(value))
        data = 0
    return struct.pack('<b', data)

def write_uint16_t(data):
    """ data is type uint16_t """
    if data < 0 or data > 65535:
        print("write_uint16_t value error: {}".format(value))
        data = 0
    return struct.pack('<h', data)

def write_uint32_t(data):
    """ data is type uint32_t """
    if data < 0 or data > 4294967295:
        print("write_uint32_t value error: {}".format(value))
        data = 0
    return struct.pack('<l', data)


def calculate_checksum(data):
    ch_a, ch_b = 0, 0
    for i in range(len(data)):
        ch_a = ( ch_a + ord(data[i]) )
        ch_b = ( ch_b + ch_a )
    ch_a = ch_a & 0xff
    ch_b = ch_b  & 0xff
    return [ch_a, ch_b]


def data_loop(serial, display_callback=None, samples=None):

    new_data = False

    good_count = 0
    total_count = 0

    data_count = 0

    header_match = False
    incoming_sample = sampleStruct()

    process_state = 0
    payloadSize = 0
    _msgType = 0
    _buffer = []
    checksum = []

    all_data = []

    if display_callback:
        display_coms = Communicate()
        display_coms.data_signal.connect(display_callback)

    while True:

        while serial.in_waiting:
            c = serial.read(1)
            # print("c :", process_state, c, type(c))
            if ( process_state < 2):
                if ( c == b't'):  # match to b't' == 0x74
                    process_state += 1
                else:
                    process_state = 0
            else:
                # Header match
                # Now cache the message type
                if ( process_state == 2 ):
                    header_match = True
                    # print("header match")
                    # This a a uint8 byte, unpack returns tuple so we have to get the first index
                    _msgType = struct.unpack('<b', c)[0]
                    payloadSize = 0
                    _buffer = []
                    # Set the payload size
                    if ( _msgType == MSGT_SAMPLE ):
                        total_count += 1
                        payloadSize = 11
                    else:
                        print("UNKNOWN MSG TYPE")
                        process_state = 0
                        _buffer = []
                
                # Buffer the payload
                if ( ( process_state-2 ) < payloadSize ):
                    # print("buffering state= {}, payloadsize= {}".format(process_state, payloadSize))
                    _buffer.append(c)

                process_state += 1

                # End of payload, calculate checksum
                if ( process_state == (payloadSize+3) ):
                    # Last byte, so we can do the checksum
                    # print("buffer :", _buffer)
                    checksum = calculate_checksum(_buffer)
                    # print("state=", process_state, "checksum output :", checksum)
                    if ( checksum[0] != ord(c) ):
                        # print("checksum A failed", checksum[0], ord(c), c)
                        process_state = 0
                    # else:
                    #     print("checksum A passed", checksum[0], ord(c), c)
                elif ( process_state == (payloadSize+4) ):
                    if ( checksum[1] == ord(c) ):
                        # print("checksum B passed", checksum[1], ord(c), c)
                        good_count +=1
                        new_data = True
                    # else:
                    #     print("checksum B failed", checksum[1], ord(c), c)
                    process_state = 0
                elif ( process_state > (payloadSize+5) ):
                    process_state = 0


        if new_data:        
            # print("NEW DATA")
            new_data = False

            if _msgType == MSGT_SAMPLE:
                # print("NEW SAMPLE STRUCT")
                data_count += 1
                incoming_sample.load = read_unit32_t(_buffer[1:5])
                incoming_sample.steps = read_unit32_t(_buffer[5:9])
                incoming_sample.displacement = read_unit16_t(_buffer[9:11])
                all_data.append(copy.deepcopy(incoming_sample))
                print(incoming_sample)
                if display_callback:
                    display_coms.data_signal.emit(incoming_sample.load)
                

        if (total_count == 150) :
            print("{}/{}. {} dropped. {:.2f}%".format(good_count, total_count, total_count-good_count, 100*good_count/total_count))
            total_count = 0
            good_count = 0

        if samples:
            if data_count >= samples:
                return all_data


if __name__ == "__main__":
    import sys
    import threading
    from PyQt5.QtWidgets import *
    from PyQt5.QtCore import *
    from PyQt5.QtGui import *

    from display import CustomMainWindow, Communicate

    arduino = start_serial("COM4")

    app = QApplication(sys.argv)
    QApplication.setStyle(QStyleFactory.create('Plastique'))
    myGUI = CustomMainWindow()
    myDataLoop = threading.Thread(name='data_loop', target=data_loop, daemon=True, args=(arduino, myGUI.addData_callbackFunc,))
    myDataLoop.start()
    sys.exit(app.exec_())

