from time import sleep
import serial
import numpy as np
from threading import Lock
from collections import deque

class SerialProtocol:
    def __init__(self, device = "/dev/ttyACM1", baud=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout = None, endian = "little") -> None:
        self.serial_dev = serial.Serial(device, baud, parity = parity, stopbits = stopbits, timeout = timeout)
        self.running = True
        self.ROS_HEADER_LENGTH = 7
        self.data_dict = {}
        self.serializer_dict = {}
        self.endianness = endian
        self.bytes_buffer = bytearray()

    def checksum(self, addends):
        sum = np.sum(addends)
        return 255 - (sum % 256)

    def get_cur_topic_data(self, topic_id):
        lock = self.data_dict[topic_id][0]
        lock.acquire()
        to_return = np.copy(self.data_dict[topic_id][1])
        lock.release()
        return to_return

    def send_topic_data(self, topic_id, data):
        to_send = bytearray(b'\xff\xfe')
        serializer_fn = self.serializer_dict[topic_id][1]
        serialized_data = serializer_fn(data)
        msg_len = len(serialized_data)
        msg_len_low = msg_len % 0xff
        msg_len_high = msg_len >> 8
        to_send.append(msg_len_low)
        to_send.append(msg_len_high)
        cs_1 = self.checksum([msg_len_low, msg_len_high])
        to_send.append(cs_1)
        topic_id_low =  topic_id % 0xff
        topic_id_high = topic_id >> 8
        to_send.append(topic_id_low)
        to_send.append(topic_id_high)
        cs_2_addends = np.array([topic_id_low, topic_id_high])
        for entry in serialized_data:
            to_send.append(entry)
            cs_2_addends = np.append(cs_2_addends, entry)
        cs_2 = self.checksum(cs_2_addends)
        to_send.append(cs_2)
        self.serial_dev.write(to_send)

    def pop_n_bytes_buffer(self, n):
        to_return = bytearray(n)
        # if we dont have the number of bytes we want yet, spin
        while(len(self.bytes_buffer) < n):
            pass
        to_return[:] = self.bytes_buffer[:n]
        del self.bytes_buffer[:n]
        return to_return

    @staticmethod
    def read_serial_loop(protocol):
        while(protocol.running):
            bytes_in_buffer = protocol.serial_dev.in_waiting
            if(bytes_in_buffer > 0):
                in_bytes = bytearray(protocol.serial_dev.read(bytes_in_buffer))
                protocol.bytes_buffer.extend(in_bytes)


    @staticmethod
    def parse_loop(protocol):
        while(protocol.running):
            valid_header = True
            valid_message = True
            header_data = np.zeros(protocol.ROS_HEADER_LENGTH, dtype=np.int)

            # wait for initial sync byte
            while(header_data[0] != 0xff):
                read_result = protocol.pop_n_bytes_buffer(1) #protocol.serial_dev.read()
                header_data[0] = int.from_bytes(read_result, protocol.endianness)
            # read the reset of the header. If we dont read enough due to timeout,
            # declare the header as invalid and continue
            header_timeout_bytes = protocol.pop_n_bytes_buffer(protocol.ROS_HEADER_LENGTH - 1) #protocol.serial_dev.read(protocol.ROS_HEADER_LENGTH - 1)
            if(len(header_timeout_bytes) != (protocol.ROS_HEADER_LENGTH - 1)):
                valid_header = False
                print("invalid header A")

            # if we read the entire header properly, then validate it
            if(valid_header):
                header_data[1:] = [byte for byte in header_timeout_bytes]

                # check the sync flag/protocol version
                valid_header = valid_header and (header_data[1] == 0xfe);

                # compute the checksum on the received message length
                #and compare it to the received checksum
                cs1_addends = [header_data[2], header_data[3]]
                cs_msg_len = protocol.checksum(cs1_addends)
                valid_header = valid_header and (cs_msg_len == header_data[4])
                if(not valid_header):
                    print("invalid header B")

            # if we get in here, then we consider our header valid
            if(valid_header):
                message_len = (header_data[3] << 8) + header_data[2]
                topic_id = (header_data[6] << 8) + header_data[5]
                msg_data_serialzied = protocol.pop_n_bytes_buffer(message_len) #protocol.serial_dev.read(message_len)
                if(len(msg_data_serialzied) != message_len):
                    valid_message = False
                    print("invalid message A")
                topic_msg_data_checksum =  int.from_bytes(protocol.pop_n_bytes_buffer(1), protocol.endianness) #int.from_bytes(protocol.serial_dev.read(), protocol.endianness)
                cs2_addends = np.zeros(message_len + 2, dtype=np.int)
                cs2_addends[0:2] = header_data[5:7]
                cs2_addends[2:] = [byte for byte in msg_data_serialzied]
                cs_topic_msg_data = protocol.checksum(cs2_addends)

                valid_message = valid_message & (cs_topic_msg_data == topic_msg_data_checksum)
                if(not valid_message):
                    print("invalid message B")

                # if we get in here, both message and header were valid
                if(valid_message):
                    if(topic_id in protocol.serializer_dict):
                        deserialize_fn = protocol.serializer_dict[topic_id][0]
                        cb_fn = None
                        if(len(protocol.serializer_dict[topic_id]) == 3 and protocol.serializer_dict[topic_id][2] is not None):
                            cb_fn = protocol.serializer_dict[topic_id][2]

                        if(not topic_id in protocol.data_dict):
                            new_lock = Lock()
                            received_obj = deserialize_fn(msg_data_serialzied)
                            protocol.data_dict[topic_id] = [new_lock, received_obj]
                            if(cb_fn is not None):
                                cb_fn(received_obj)
                        else:
                            lock = protocol.data_dict[topic_id][0]
                            lock.acquire()
                            received_obj = deserialize_fn(msg_data_serialzied)
                            protocol.data_dict[topic_id][1] = received_obj
                            if(cb_fn is not None):
                                cb_fn(received_obj)
                            lock.release()

            header_data = np.zeros(protocol.ROS_HEADER_LENGTH)
            sleep(0.001)