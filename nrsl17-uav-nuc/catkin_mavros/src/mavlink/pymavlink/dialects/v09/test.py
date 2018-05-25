'''
MAVLink protocol implementation (auto-generated by mavgen.py)

Generated from: test.xml

Note: this file has been auto-generated. DO NOT EDIT
'''

import struct, array, time, json, os, sys, platform

from ...generator.mavcrc import x25crc

WIRE_PROTOCOL_VERSION = "0.9"
DIALECT = "test"

native_supported = platform.system() != 'Windows' # Not yet supported on other dialects
native_force = 'MAVNATIVE_FORCE' in os.environ # Will force use of native code regardless of what client app wants
native_testing = 'MAVNATIVE_TESTING' in os.environ # Will force both native and legacy code to be used and their results compared

if native_supported:
    try:
        import mavnative
    except ImportError:
        print("ERROR LOADING MAVNATIVE - falling back to python implementation")
        native_supported = False

# some base types from mavlink_types.h
MAVLINK_TYPE_CHAR     = 0
MAVLINK_TYPE_UINT8_T  = 1
MAVLINK_TYPE_INT8_T   = 2
MAVLINK_TYPE_UINT16_T = 3
MAVLINK_TYPE_INT16_T  = 4
MAVLINK_TYPE_UINT32_T = 5
MAVLINK_TYPE_INT32_T  = 6
MAVLINK_TYPE_UINT64_T = 7
MAVLINK_TYPE_INT64_T  = 8
MAVLINK_TYPE_FLOAT    = 9
MAVLINK_TYPE_DOUBLE   = 10


class MAVLink_header(object):
    '''MAVLink message header'''
    def __init__(self, msgId, mlen=0, seq=0, srcSystem=0, srcComponent=0):
        self.mlen = mlen
        self.seq = seq
        self.srcSystem = srcSystem
        self.srcComponent = srcComponent
        self.msgId = msgId

    def pack(self):
        return struct.pack('BBBBBB', 85, self.mlen, self.seq,
                          self.srcSystem, self.srcComponent, self.msgId)

class MAVLink_message(object):
    '''base MAVLink message class'''
    def __init__(self, msgId, name):
        self._header     = MAVLink_header(msgId)
        self._payload    = None
        self._msgbuf     = None
        self._crc        = None
        self._fieldnames = []
        self._type       = name

    def get_msgbuf(self):
        if isinstance(self._msgbuf, bytearray):
            return self._msgbuf
        return bytearray(self._msgbuf)

    def get_header(self):
        return self._header

    def get_payload(self):
        return self._payload

    def get_crc(self):
        return self._crc

    def get_fieldnames(self):
        return self._fieldnames

    def get_type(self):
        return self._type

    def get_msgId(self):
        return self._header.msgId

    def get_srcSystem(self):
        return self._header.srcSystem

    def get_srcComponent(self):
        return self._header.srcComponent

    def get_seq(self):
        return self._header.seq

    def __str__(self):
        ret = '%s {' % self._type
        for a in self._fieldnames:
            v = getattr(self, a)
            ret += '%s : %s, ' % (a, v)
        ret = ret[0:-2] + '}'
        return ret

    def __ne__(self, other):
        return not self.__eq__(other)

    def __eq__(self, other):
        if other == None:
            return False

        if self.get_type() != other.get_type():
            return False

        # We do not compare CRC because native code doesn't provide it
        #if self.get_crc() != other.get_crc():
        #    return False

        if self.get_seq() != other.get_seq():
            return False

        if self.get_srcSystem() != other.get_srcSystem():
            return False            

        if self.get_srcComponent() != other.get_srcComponent():
            return False   
            
        for a in self._fieldnames:
            if getattr(self, a) != getattr(other, a):
                return False

        return True

    def to_dict(self):
        d = dict({})
        d['mavpackettype'] = self._type
        for a in self._fieldnames:
          d[a] = getattr(self, a)
        return d

    def to_json(self):
        return json.dumps(self.to_dict())

    def pack(self, mav, crc_extra, payload):
        self._payload = payload
        self._header  = MAVLink_header(self._header.msgId, len(payload), mav.seq,
                                       mav.srcSystem, mav.srcComponent)
        self._msgbuf = self._header.pack() + payload
        crc = x25crc(self._msgbuf[1:])
        if False: # using CRC extra
            crc.accumulate_str(struct.pack('B', crc_extra))
        self._crc = crc.crc
        self._msgbuf += struct.pack('<H', self._crc)
        return self._msgbuf


# enums

class EnumEntry(object):
    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.param = {}
        
enums = {}

# message IDs
MAVLINK_MSG_ID_BAD_DATA = -1
MAVLINK_MSG_ID_TEST_TYPES = 0

class MAVLink_test_types_message(MAVLink_message):
        '''
        Test all field types
        '''
        id = MAVLINK_MSG_ID_TEST_TYPES
        name = 'TEST_TYPES'
        fieldnames = ['c', 's', 'u8', 'u16', 'u32', 'u64', 's8', 's16', 's32', 's64', 'f', 'd', 'u8_array', 'u16_array', 'u32_array', 'u64_array', 's8_array', 's16_array', 's32_array', 's64_array', 'f_array', 'd_array']
        ordered_fieldnames = [ 'c', 's', 'u8', 'u16', 'u32', 'u64', 's8', 's16', 's32', 's64', 'f', 'd', 'u8_array', 'u16_array', 'u32_array', 'u64_array', 's8_array', 's16_array', 's32_array', 's64_array', 'f_array', 'd_array' ]
        format = '>c10sBHIQbhiqfd3B3H3I3Q3b3h3i3q3f3d'
        native_format = bytearray('>ccBHIQbhiqfdBHIQbhiqfd', 'ascii')
        orders = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
        lengths = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
        array_lengths = [0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3]
        crc_extra = 91

        def __init__(self, c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array):
                MAVLink_message.__init__(self, MAVLink_test_types_message.id, MAVLink_test_types_message.name)
                self._fieldnames = MAVLink_test_types_message.fieldnames
                self.c = c
                self.s = s
                self.u8 = u8
                self.u16 = u16
                self.u32 = u32
                self.u64 = u64
                self.s8 = s8
                self.s16 = s16
                self.s32 = s32
                self.s64 = s64
                self.f = f
                self.d = d
                self.u8_array = u8_array
                self.u16_array = u16_array
                self.u32_array = u32_array
                self.u64_array = u64_array
                self.s8_array = s8_array
                self.s16_array = s16_array
                self.s32_array = s32_array
                self.s64_array = s64_array
                self.f_array = f_array
                self.d_array = d_array

        def pack(self, mav):
                return MAVLink_message.pack(self, mav, 91, struct.pack('>c10sBHIQbhiqfd3B3H3I3Q3b3h3i3q3f3d', self.c, self.s, self.u8, self.u16, self.u32, self.u64, self.s8, self.s16, self.s32, self.s64, self.f, self.d, self.u8_array[0], self.u8_array[1], self.u8_array[2], self.u16_array[0], self.u16_array[1], self.u16_array[2], self.u32_array[0], self.u32_array[1], self.u32_array[2], self.u64_array[0], self.u64_array[1], self.u64_array[2], self.s8_array[0], self.s8_array[1], self.s8_array[2], self.s16_array[0], self.s16_array[1], self.s16_array[2], self.s32_array[0], self.s32_array[1], self.s32_array[2], self.s64_array[0], self.s64_array[1], self.s64_array[2], self.f_array[0], self.f_array[1], self.f_array[2], self.d_array[0], self.d_array[1], self.d_array[2]))


mavlink_map = {
        MAVLINK_MSG_ID_TEST_TYPES : MAVLink_test_types_message,
}

class MAVError(Exception):
        '''MAVLink error class'''
        def __init__(self, msg):
            Exception.__init__(self, msg)
            self.message = msg

class MAVString(str):
        '''NUL terminated string'''
        def __init__(self, s):
                str.__init__(self)
        def __str__(self):
            i = self.find(chr(0))
            if i == -1:
                return self[:]
            return self[0:i]

class MAVLink_bad_data(MAVLink_message):
        '''
        a piece of bad data in a mavlink stream
        '''
        def __init__(self, data, reason):
                MAVLink_message.__init__(self, MAVLINK_MSG_ID_BAD_DATA, 'BAD_DATA')
                self._fieldnames = ['data', 'reason']
                self.data = data
                self.reason = reason
                self._msgbuf = data

        def __str__(self):
            '''Override the __str__ function from MAVLink_messages because non-printable characters are common in to be the reason for this message to exist.'''
            return '%s {%s, data:%s}' % (self._type, self.reason, [('%x' % ord(i) if isinstance(i, str) else '%x' % i) for i in self.data])

class MAVLink(object):
        '''MAVLink protocol handling class'''
        def __init__(self, file, srcSystem=0, srcComponent=0, use_native=False):
                self.seq = 0
                self.file = file
                self.srcSystem = srcSystem
                self.srcComponent = srcComponent
                self.callback = None
                self.callback_args = None
                self.callback_kwargs = None
                self.send_callback = None
                self.send_callback_args = None
                self.send_callback_kwargs = None
                self.buf = bytearray()
                self.expected_length = 8
                self.have_prefix_error = False
                self.robust_parsing = False
                self.protocol_marker = 85
                self.little_endian = False
                self.crc_extra = False
                self.sort_fields = False
                self.total_packets_sent = 0
                self.total_bytes_sent = 0
                self.total_packets_received = 0
                self.total_bytes_received = 0
                self.total_receive_errors = 0
                self.startup_time = time.time()
                if native_supported and (use_native or native_testing or native_force):
                    print("NOTE: mavnative is currently beta-test code")
                    self.native = mavnative.NativeConnection(MAVLink_message, mavlink_map)
                else:
                    self.native = None
                if native_testing:
                    self.test_buf = bytearray()

        def set_callback(self, callback, *args, **kwargs):
            self.callback = callback
            self.callback_args = args
            self.callback_kwargs = kwargs

        def set_send_callback(self, callback, *args, **kwargs):
            self.send_callback = callback
            self.send_callback_args = args
            self.send_callback_kwargs = kwargs

        def send(self, mavmsg):
                '''send a MAVLink message'''
                buf = mavmsg.pack(self)
                self.file.write(buf)
                self.seq = (self.seq + 1) % 256
                self.total_packets_sent += 1
                self.total_bytes_sent += len(buf)
                if self.send_callback:
                    self.send_callback(mavmsg, *self.send_callback_args, **self.send_callback_kwargs)

        def bytes_needed(self):
            '''return number of bytes needed for next parsing stage'''
            if self.native:
                ret = self.native.expected_length - len(self.buf)
            else:
                ret = self.expected_length - len(self.buf)
            
            if ret <= 0:
                return 1
            return ret

        def __parse_char_native(self, c):
            '''this method exists only to see in profiling results'''
            m = self.native.parse_chars(c)
            return m

        def __callbacks(self, msg):
            '''this method exists only to make profiling results easier to read'''
            if self.callback:
                self.callback(msg, *self.callback_args, **self.callback_kwargs)

        def parse_char(self, c):
            '''input some data bytes, possibly returning a new message'''
            self.buf.extend(c)

            self.total_bytes_received += len(c)

            if self.native:
                if native_testing:
                    self.test_buf.extend(c)
                    m = self.__parse_char_native(self.test_buf)
                    m2 = self.__parse_char_legacy()
                    if m2 != m:
                        print("Native: %s\nLegacy: %s\n" % (m, m2))
                        raise Exception('Native vs. Legacy mismatch')
                else:
                    m = self.__parse_char_native(self.buf)
            else:
                m = self.__parse_char_legacy()

            if m != None:
                self.total_packets_received += 1
                self.__callbacks(m)

            return m

        def __parse_char_legacy(self):
            '''input some data bytes, possibly returning a new message (uses no native code)'''
            if len(self.buf) >= 1 and self.buf[0] != 85:
                magic = self.buf[0]
                self.buf = self.buf[1:]
                if self.robust_parsing:
                    m = MAVLink_bad_data(chr(magic), "Bad prefix")
                    self.expected_length = 8
                    self.total_receive_errors += 1
                    return m
                if self.have_prefix_error:
                    return None
                self.have_prefix_error = True
                self.total_receive_errors += 1
                raise MAVError("invalid MAVLink prefix '%s'" % magic)
            self.have_prefix_error = False
            if len(self.buf) >= 2:
                if sys.version_info[0] < 3:
                    (magic, self.expected_length) = struct.unpack('BB', str(self.buf[0:2])) # bytearrays are not supported in py 2.7.3
                else:
                    (magic, self.expected_length) = struct.unpack('BB', self.buf[0:2])
                self.expected_length += 8
            if self.expected_length >= 8 and len(self.buf) >= self.expected_length:
                mbuf = array.array('B', self.buf[0:self.expected_length])
                self.buf = self.buf[self.expected_length:]
                self.expected_length = 8
                if self.robust_parsing:
                    try:
                        m = self.decode(mbuf)
                    except MAVError as reason:
                        m = MAVLink_bad_data(mbuf, reason.message)
                        self.total_receive_errors += 1
                else:
                    m = self.decode(mbuf)
                return m
            return None

        def parse_buffer(self, s):
            '''input some data bytes, possibly returning a list of new messages'''
            m = self.parse_char(s)
            if m is None:
                return None
            ret = [m]
            while True:
                m = self.parse_char("")
                if m is None:
                    return ret
                ret.append(m)
            return ret

        def decode(self, msgbuf):
                '''decode a buffer as a MAVLink message'''
                # decode the header
                try:
                    magic, mlen, seq, srcSystem, srcComponent, msgId = struct.unpack('cBBBBB', msgbuf[:6])
                except struct.error as emsg:
                    raise MAVError('Unable to unpack MAVLink header: %s' % emsg)
                if ord(magic) != 85:
                    raise MAVError("invalid MAVLink prefix '%s'" % magic)
                if mlen != len(msgbuf)-8:
                    raise MAVError('invalid MAVLink message length. Got %u expected %u, msgId=%u' % (len(msgbuf)-8, mlen, msgId))

                if not msgId in mavlink_map:
                    raise MAVError('unknown MAVLink message ID %u' % msgId)

                # decode the payload
                type = mavlink_map[msgId]
                fmt = type.format
                order_map = type.orders
                len_map = type.lengths
                crc_extra = type.crc_extra

                # decode the checksum
                try:
                    crc, = struct.unpack('<H', msgbuf[-2:])
                except struct.error as emsg:
                    raise MAVError('Unable to unpack MAVLink CRC: %s' % emsg)
                crcbuf = msgbuf[1:-2]
                if False: # using CRC extra
                    crcbuf.append(crc_extra)
                crc2 = x25crc(crcbuf)
                if crc != crc2.crc:
                    raise MAVError('invalid MAVLink CRC in msgID %u 0x%04x should be 0x%04x' % (msgId, crc, crc2.crc))

                try:
                    t = struct.unpack(fmt, msgbuf[6:-2])
                except struct.error as emsg:
                    raise MAVError('Unable to unpack MAVLink payload type=%s fmt=%s payloadLength=%u: %s' % (
                        type, fmt, len(msgbuf[6:-2]), emsg))

                tlist = list(t)
                # handle sorted fields
                if False:
                    t = tlist[:]
                    if sum(len_map) == len(len_map):
                        # message has no arrays in it
                        for i in range(0, len(tlist)):
                            tlist[i] = t[order_map[i]]
                    else:
                        # message has some arrays
                        tlist = []
                        for i in range(0, len(order_map)):
                            order = order_map[i]
                            L = len_map[order]
                            tip = sum(len_map[:order])
                            field = t[tip]
                            if L == 1 or isinstance(field, str):
                                tlist.append(field)
                            else:
                                tlist.append(t[tip:(tip + L)])

                # terminate any strings
                for i in range(0, len(tlist)):
                    if isinstance(tlist[i], str):
                        tlist[i] = str(MAVString(tlist[i]))
                t = tuple(tlist)
                # construct the message object
                try:
                    m = type(*t)
                except Exception as emsg:
                    raise MAVError('Unable to instantiate MAVLink message of type %s : %s' % (type, emsg))
                m._msgbuf = msgbuf
                m._payload = msgbuf[6:-2]
                m._crc = crc
                m._header = MAVLink_header(msgId, mlen, seq, srcSystem, srcComponent)
                return m
        def test_types_encode(self, c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array):
                '''
                Test all field types

                c                         : char (char)
                s                         : string (char)
                u8                        : uint8_t (uint8_t)
                u16                       : uint16_t (uint16_t)
                u32                       : uint32_t (uint32_t)
                u64                       : uint64_t (uint64_t)
                s8                        : int8_t (int8_t)
                s16                       : int16_t (int16_t)
                s32                       : int32_t (int32_t)
                s64                       : int64_t (int64_t)
                f                         : float (float)
                d                         : double (double)
                u8_array                  : uint8_t_array (uint8_t)
                u16_array                 : uint16_t_array (uint16_t)
                u32_array                 : uint32_t_array (uint32_t)
                u64_array                 : uint64_t_array (uint64_t)
                s8_array                  : int8_t_array (int8_t)
                s16_array                 : int16_t_array (int16_t)
                s32_array                 : int32_t_array (int32_t)
                s64_array                 : int64_t_array (int64_t)
                f_array                   : float_array (float)
                d_array                   : double_array (double)

                '''
                return MAVLink_test_types_message(c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array)

        def test_types_send(self, c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array):
                '''
                Test all field types

                c                         : char (char)
                s                         : string (char)
                u8                        : uint8_t (uint8_t)
                u16                       : uint16_t (uint16_t)
                u32                       : uint32_t (uint32_t)
                u64                       : uint64_t (uint64_t)
                s8                        : int8_t (int8_t)
                s16                       : int16_t (int16_t)
                s32                       : int32_t (int32_t)
                s64                       : int64_t (int64_t)
                f                         : float (float)
                d                         : double (double)
                u8_array                  : uint8_t_array (uint8_t)
                u16_array                 : uint16_t_array (uint16_t)
                u32_array                 : uint32_t_array (uint32_t)
                u64_array                 : uint64_t_array (uint64_t)
                s8_array                  : int8_t_array (int8_t)
                s16_array                 : int16_t_array (int16_t)
                s32_array                 : int32_t_array (int32_t)
                s64_array                 : int64_t_array (int64_t)
                f_array                   : float_array (float)
                d_array                   : double_array (double)

                '''
                return self.send(self.test_types_encode(c, s, u8, u16, u32, u64, s8, s16, s32, s64, f, d, u8_array, u16_array, u32_array, u64_array, s8_array, s16_array, s32_array, s64_array, f_array, d_array))

