from ctypes import *
import logging
import os
import numpy as np

try:
    from itertools import izip
except ImportError:
    izip = zip

path = os.path.dirname(__file__)
logging.basicConfig()
logger = logging.getLogger('HackRf Core')
logger.setLevel(logging.DEBUG)

libhackrf = CDLL('/usr/local/lib/libhackrf.so')

def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)

HackRfVendorRequest = enum(
    HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE=1,
    HACKRF_VENDOR_REQUEST_MAX2837_WRITE=2,
    HACKRF_VENDOR_REQUEST_MAX2837_READ=3,
    HACKRF_VENDOR_REQUEST_SI5351C_WRITE=4,
    HACKRF_VENDOR_REQUEST_SI5351C_READ=5,
    HACKRF_VENDOR_REQUEST_SAMPLE_RATE_SET=6,
    HACKRF_VENDOR_REQUEST_BASEBAND_FILTER_BANDWIDTH_SET=7,
    HACKRF_VENDOR_REQUEST_RFFC5071_WRITE=8,
    HACKRF_VENDOR_REQUEST_RFFC5071_READ=9,
    HACKRF_VENDOR_REQUEST_SPIFLASH_ERASE=10,
    HACKRF_VENDOR_REQUEST_SPIFLASH_WRITE=11,
    HACKRF_VENDOR_REQUEST_SPIFLASH_READ=12,
    HACKRF_VENDOR_REQUEST_CPLD_WRITE=13,
    HACKRF_VENDOR_REQUEST_BOARD_ID_READ=14,
    HACKRF_VENDOR_REQUEST_VERSION_STRING_READ=15,
    HACKRF_VENDOR_REQUEST_SET_FREQ=16,
    HACKRF_VENDOR_REQUEST_AMP_ENABLE=17,
    HACKRF_VENDOR_REQUEST_BOARD_PARTID_SERIALNO_READ=18,
    HACKRF_VENDOR_REQUEST_SET_LNA_GAIN=19,
    HACKRF_VENDOR_REQUEST_SET_VGA_GAIN=20,
    HACKRF_VENDOR_REQUEST_SET_TXVGA_GAIN=21)

HackRfConstants = enum(
    LIBUSB_ENDPOINT_IN=0x80,
    LIBUSB_ENDPOINT_OUT=0x00,
    HACKRF_DEVICE_OUT=0x40,
    HACKRF_DEVICE_IN=0xC0,
    HACKRF_USB_VID=0x1d50,
    HACKRF_USB_PID=0x6089)

HackRfError = enum(
    HACKRF_SUCCESS=0,
    HACKRF_TRUE=1,
    HACKRF_ERROR_INVALID_PARAM=-2,
    HACKRF_ERROR_NOT_FOUND=-5,
    HACKRF_ERROR_BUSY=-6,
    HACKRF_ERROR_NO_MEM=-11,
    HACKRF_ERROR_LIBUSB=-1000,
    HACKRF_ERROR_THREAD=-1001,
    HACKRF_ERROR_STREAMING_THREAD_ERR=-1002,
    HACKRF_ERROR_STREAMING_STOPPED=-1003,
    HACKRF_ERROR_STREAMING_EXIT_CALLED=-1004,
    HACKRF_ERROR_OTHER=-9999,
    # Python defaults to returning none
    HACKRF_ERROR=None)

HackRfTranscieverMode = enum(
    HACKRF_TRANSCEIVER_MODE_OFF=0,
    HACKRF_TRANSCEIVER_MODE_RECEIVE=1,
    HACKRF_TRANSCEIVER_MODE_TRANSMIT=2)

# Data structures
_libusb_device_handle = c_void_p
_pthread_t = c_ulong

class hackrf_device(Structure):
    pass

class hackrf_transfer(Structure):
        _fields_ = [("hackrf_device", POINTER(hackrf_device)),
                ("buffer", POINTER(c_byte)),
                ("buffer_length", c_int),
                ("valid_length", c_int),
                ("rx_ctx", c_void_p),
                ("tx_ctx", c_void_p) ]

_callback = CFUNCTYPE(c_int, POINTER(hackrf_transfer))

hackrf_device._fields_ = [("usb_device", POINTER(_libusb_device_handle)),
        ("transfers", POINTER(POINTER(hackrf_transfer))),
        ("callback", _callback),
        ("transfer_thread_started", c_int),
        ("transfer_thread", _pthread_t),
        ("transfer_count", c_uint32),
        ("buffer_size", c_uint32),
        ("streaming", c_int),
        ("rx_ctx", c_void_p),
        ("tx_ctx", c_void_p) ]

# extern ADDAPI int ADDCALL hackrf_init();
libhackrf.hackrf_init.restype = c_int
libhackrf.hackrf_init.argtypes = []
# extern ADDAPI int ADDCALL hackrf_exit();
libhackrf.hackrf_exit.restype = c_int
libhackrf.hackrf_exit.argtypes = []
# extern ADDAPI int ADDCALL hackrf_open(hackrf_device** device);
libhackrf.hackrf_open.restype = c_int
libhackrf.hackrf_open.argtypes = [POINTER(POINTER(hackrf_device))]
# extern ADDAPI int ADDCALL hackrf_close(hackrf_device* device);
libhackrf.hackrf_close.restype = c_int
libhackrf.hackrf_close.argtypes = [POINTER(hackrf_device)]
# extern ADDAPI int ADDCALL hackrf_start_rx(hackrf_device* device,
# hackrf_sample_block_cb_fn callback, void* rx_ctx);
libhackrf.hackrf_start_rx.restype = c_int
libhackrf.hackrf_start_rx.argtypes = [POINTER(hackrf_device), _callback, c_void_p]
# extern ADDAPI int ADDCALL hackrf_stop_rx(hackrf_device* device);
libhackrf.hackrf_stop_rx.restype = c_int
libhackrf.hackrf_stop_rx.argtypes = [POINTER(hackrf_device)]
# extern ADDAPI int ADDCALL hackrf_start_tx(hackrf_device* device,
# hackrf_sample_block_cb_fn callback, void* tx_ctx);
libhackrf.hackrf_start_tx.restype = c_int
libhackrf.hackrf_start_tx.argtypes = [POINTER(hackrf_device), _callback, c_void_p]
# extern ADDAPI int ADDCALL hackrf_stop_tx(hackrf_device* device);
libhackrf.hackrf_stop_tx.restype = c_int
libhackrf.hackrf_stop_tx.argtypes = [POINTER(hackrf_device)]
# extern ADDAPI int ADDCALL hackrf_is_streaming(hackrf_device* device);
libhackrf.hackrf_is_streaming.restype = c_int
libhackrf.hackrf_is_streaming.argtypes = [POINTER(hackrf_device)]
# extern ADDAPI int ADDCALL hackrf_max2837_read(hackrf_device* device,
# uint8_t register_number, uint16_t* value);
libhackrf.hackrf_max2837_read.restype = c_int
libhackrf.hackrf_max2837_read.argtypes = [
    POINTER(hackrf_device), c_uint8, POINTER(c_uint16)]
# extern ADDAPI int ADDCALL hackrf_max2837_write(hackrf_device* device,
# uint8_t register_number, uint16_t value);
libhackrf.hackrf_max2837_write.restype = c_int
libhackrf.hackrf_max2837_write.argtypes = [POINTER(hackrf_device), c_uint8, c_uint16]
# extern ADDAPI int ADDCALL hackrf_si5351c_read(hackrf_device* device,
# uint16_t register_number, uint16_t* value);
libhackrf.hackrf_si5351c_read.restype = c_int
libhackrf.hackrf_si5351c_read.argtypes = [
    POINTER(hackrf_device), c_uint16, POINTER(c_uint16)]
# extern ADDAPI int ADDCALL hackrf_si5351c_write(hackrf_device* device,
# uint16_t register_number, uint16_t value);
libhackrf.hackrf_si5351c_write.restype = c_int
libhackrf.hackrf_si5351c_write.argtypes = [POINTER(hackrf_device), c_uint16, c_uint16]
# extern ADDAPI int ADDCALL
# hackrf_set_baseband_filter_bandwidth(hackrf_device* device, const
# uint32_t bandwidth_hz);
libhackrf.hackrf_set_baseband_filter_bandwidth.restype = c_int
libhackrf.hackrf_set_baseband_filter_bandwidth.argtypes = [
    POINTER(hackrf_device), c_uint32]
# extern ADDAPI int ADDCALL hackrf_rffc5071_read(hackrf_device* device,
# uint8_t register_number, uint16_t* value);
libhackrf.hackrf_rffc5071_read.restype = c_int
libhackrf.hackrf_rffc5071_read.argtypes = [
    POINTER(hackrf_device), c_uint8, POINTER(c_uint16)]
# extern ADDAPI int ADDCALL hackrf_rffc5071_write(hackrf_device*
# device, uint8_t register_number, uint16_t value);
libhackrf.hackrf_rffc5071_write.restype = c_int
libhackrf.hackrf_rffc5071_write.argtypes = [POINTER(hackrf_device), c_uint8, c_uint16]
# extern ADDAPI int ADDCALL hackrf_spiflash_erase(hackrf_device*
# device);
libhackrf.hackrf_spiflash_erase.restype = c_int
libhackrf.hackrf_spiflash_erase.argtypes = [POINTER(hackrf_device)]
# extern ADDAPI int ADDCALL hackrf_spiflash_write(hackrf_device*
# device, const uint32_t address, const uint16_t length, unsigned char*
# const data);
libhackrf.hackrf_spiflash_write.restype = c_int
libhackrf.hackrf_spiflash_write.argtypes = [
    POINTER(hackrf_device), c_uint32, c_uint16, POINTER(c_ubyte)]
# extern ADDAPI int ADDCALL hackrf_spiflash_read(hackrf_device* device,
# const uint32_t address, const uint16_t length, unsigned char* data);
libhackrf.hackrf_spiflash_read.restype = c_int
libhackrf.hackrf_spiflash_read.argtypes = [
    POINTER(hackrf_device), c_uint32, c_uint16, POINTER(c_ubyte)]
# extern ADDAPI int ADDCALL hackrf_cpld_write(hackrf_device* device,
#         unsigned char* const data, const unsigned int total_length);
libhackrf.hackrf_cpld_write.restype = c_int
libhackrf.hackrf_cpld_write.argtypes = [POINTER(hackrf_device), POINTER(c_ubyte), c_uint]
# extern ADDAPI int ADDCALL hackrf_board_id_read(hackrf_device* device,
# uint8_t* value);
libhackrf.hackrf_board_id_read.restype = c_int
libhackrf.hackrf_board_id_read.argtypes = [POINTER(hackrf_device), POINTER(c_uint8)]
# extern ADDAPI int ADDCALL hackrf_version_string_read(hackrf_device*
# device, char* version, uint8_t length);
libhackrf.hackrf_version_string_read.restype = c_int
libhackrf.hackrf_version_string_read.argtypes = [POINTER(hackrf_device), POINTER(c_char), c_uint8]
# extern ADDAPI int ADDCALL hackrf_set_freq(hackrf_device* device,
# const uint64_t freq_hz);
libhackrf.hackrf_set_freq.restype = c_int
libhackrf.hackrf_set_freq.argtypes = [POINTER(hackrf_device), c_uint64]

# extern ADDAPI int ADDCALL hackrf_set_freq_explicit(hackrf_device* device,
#         const uint64_t if_freq_hz, const uint64_t lo_freq_hz,
#         const enum rf_path_filter path);,
# libhackrf.hackrf_set_freq_explicit.restype = c_int
# libhackrf.hackrf_set_freq_explicit.argtypes = [c_uint64,
# c_uint64, ]

# extern ADDAPI int ADDCALL
# hackrf_set_sample_rate_manual(hackrf_device* device, const uint32_t
# freq_hz, const uint32_t divider);
libhackrf.hackrf_set_sample_rate_manual.restype = c_int
libhackrf.hackrf_set_sample_rate_manual.argtypes = [
    POINTER(hackrf_device), c_uint32, c_uint32]
# extern ADDAPI int ADDCALL hackrf_set_sample_rate(hackrf_device*
# device, const double freq_hz);
libhackrf.hackrf_set_sample_rate.restype = c_int
libhackrf.hackrf_set_sample_rate.argtypes = [POINTER(hackrf_device), c_double]
# extern ADDAPI int ADDCALL hackrf_set_amp_enable(hackrf_device*
# device, const uint8_t value);
libhackrf.hackrf_set_amp_enable.restype = c_int
libhackrf.hackrf_set_amp_enable.argtypes = [POINTER(hackrf_device), c_uint8]

# extern ADDAPI int ADDCALL
# hackrf_board_partid_serialno_read(hackrf_device* device,
# read_partid_serialno_t* read_partid_serialno);
libhackrf.hackrf_board_partid_serialno_read.restype = c_int
libhackrf.hackrf_board_partid_serialno_read.argtypes = [POINTER(hackrf_device)]

# extern ADDAPI int ADDCALL hackrf_set_lna_gain(hackrf_device* device,
# uint32_t value);
libhackrf.hackrf_set_lna_gain.restype = c_int
libhackrf.hackrf_set_lna_gain.argtypes = [POINTER(hackrf_device), c_uint32]
# extern ADDAPI int ADDCALL hackrf_set_vga_gain(hackrf_device* device,
# uint32_t value);
libhackrf.hackrf_set_vga_gain.restype = c_int
libhackrf.hackrf_set_vga_gain.argtypes = [POINTER(hackrf_device), c_uint32]
# extern ADDAPI int ADDCALL hackrf_set_txvga_gain(hackrf_device*
# device, uint32_t value);
libhackrf.hackrf_set_txvga_gain.restype = c_int
libhackrf.hackrf_set_txvga_gain.argtypes = [POINTER(hackrf_device), c_uint32]
# extern ADDAPI int ADDCALL hackrf_set_antenna_enable(hackrf_device*
# device, const uint8_t value);
libhackrf.hackrf_set_antenna_enable.restype = c_int
libhackrf.hackrf_set_antenna_enable.argtypes = [POINTER(hackrf_device), c_uint8]

# extern ADDAPI const char* ADDCALL hackrf_error_name(enum hackrf_error errcode);
# libhackrf.hackrf_error_name.restype = POINTER(c_char)
# libhackrf.hackrf_error_name.argtypes = []

# extern ADDAPI const char* ADDCALL hackrf_board_id_name(enum hackrf_board_id board_id);
# libhackrf.hackrf_board_id_name.restype = POINTER(c_char)
# libhackrf.hackrf_board_id_name.argtypes = []

# extern ADDAPI const char* ADDCALL hackrf_filter_path_name(const enum rf_path_filter path);
# libhackrf.hackrf_filter_path_name.restype = POINTER(c_char)
# libhackrf.hackrf_filter_path_name.argtypes = []

# extern ADDAPI uint32_t ADDCALL
# hackrf_compute_baseband_filter_bw_round_down_lt(const uint32_t
# bandwidth_hz);
libhackrf.hackrf_compute_baseband_filter_bw_round_down_lt.restype = c_uint32
libhackrf.hackrf_compute_baseband_filter_bw_round_down_lt.argtypes = [c_uint32]

# extern ADDAPI uint32_t ADDCALL
# hackrf_compute_baseband_filter_bw(const uint32_t bandwidth_hz);
libhackrf.hackrf_compute_baseband_filter_bw.restype = c_uint32
libhackrf.hackrf_compute_baseband_filter_bw.argtypes = [c_uint32]

class HackRf(object):
    __JELLYBEAN__ = 'Jellybean'
    __JAWBREAKER__ = 'Jawbreaker'
    __ONE = 'HackRF ONE'
    NAME_LIST = [__JELLYBEAN__, __JAWBREAKER__, __ONE]

    def __init__(self):
        self.device = POINTER(hackrf_device)()
        self.callback = None
        self.is_open = False

    def __del__(self):
        if self.is_open == True:
            self.exit()

    def setup(self):
        libhackrf.hackrf_init()
        return self.open()

    def exit(self):
        ret = self.close()
        libhackrf.hackrf_exit()
        return ret

    def open(self):
        ret = libhackrf.hackrf_open(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            self.is_open = True
            logger.debug('Successfully open HackRf device')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('No Hack Rf Detected!')

    def close(self):
        ret = libhackrf.hackrf_close(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            self.is_open = False
            logger.debug('Successfully close HackRf device')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to close!')

    def start_rx_mode(self, set_callback):
        self.callback = _callback(set_callback)
        ret = libhackrf.hackrf_start_rx(self.device, self.callback, None)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully start HackRf in Recieve Mode')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to start HackRf in Recieve Mode')

    def stop_rx_mode(self):
        ret = libhackrf.hackrf_stop_rx(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully stop HackRf in Recieve Mode')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to stop HackRf in Recieve Mode')
        return ret

    def start_tx_mode(self, set_callback):
        self.callback = _callback(set_callback)
        ret =  libhackrf.hackrf_start_tx(self.device, self.callback, None)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully start HackRf in Transfer Mode')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to start HackRf in Transfer Mode')

    def stop_tx_mode(self):
        ret = libhackrf.hackrf_stop_tx(self.device)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully stop HackRf in Transfer Mode')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to stop HackRf in Transfer Mode')

    def board_id_read(self):
        value = c_uint8()
        ret = libhackrf.hackrf_board_id_read(self.device, byref(value))
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully got Board Id')
            return value.value
        else:
            logger.error('Failed to get Board Id')

    def version_string_read(self): #POINTER(c_char), c_uint8
        version = create_string_buffer(20)
        lenth = c_uint8(20)
        ret = libhackrf.hackrf_version_string_read(self.device, version, lenth)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully got HackRf Version String')
            return version.value
        else:
            logger.error('Failed to get Version String')

    def set_freq(self, freq_hz):
        ret = libhackrf.hackrf_set_freq(self.device, freq_hz)
        if ret == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set frequency with value [%d]', freq_hz)
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Error setting frequency with value [%d]', freq_hz)

    def is_streaming(self):
        ret = libhackrf.hackrf_is_streaming(self.device)
        if(ret == 1):
            return True
        else:
            return False

    def set_lna_gain(self, value):
        ''' Sets the LNA gain, in 8Db steps, maximum value of 40 '''
        result = libhackrf.hackrf_set_lna_gain(self.device, value)
        if result == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set LNA gain to [%d]', value)
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to set LNA gain to [%d]', value)

    def set_vga_gain(self, value):
        ''' Sets the vga gain, in 2db steps, maximum value of 62 '''
        result = libhackrf.hackrf_set_vga_gain(self.device, value)
        if result == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set VGA gain to [%d]', value)
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to set VGA gain to [%d]', value)

    def set_txvga_gain(self, value):
        ''' Sets the txvga gain, in 1db steps, maximum value of 47 '''
        result = libhackrf.hackrf_set_txvga_gain(self.device, value)
        if result == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set TXVGA gain to [%d]', value)
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to set TXVGA gain to [%d]', value)


    def set_antenna_enable(self, value):
        if value == True:
            val = 1
        else:
            val = 0
        result =  libhackrf.hackrf_set_antenna_enable(self.device, val)
        if result == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set antenna_enable')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to set antenna_enable')

    def set_sample_rate(self, freq):
        result = libhackrf.hackrf_set_sample_rate(self.device, freq)
        if result != HackRfError.HACKRF_SUCCESS:
            logger.error('Error setting Sample Rate with Frequency [%d]', freq)
        else:
            logger.debug(
                'Successfully set Sample Rate with Frequency [%d]', freq)
            return HackRfError.HACKRF_SUCCESS

    def set_amp_enable(self, value):
        if value == True:
            val = 1
        else:
            val = 0
        result =  libhackrf.hackrf_set_amp_enable(self.device, val)
        if result == HackRfError.HACKRF_SUCCESS:
            logger.debug('Successfully set amp')
            return HackRfError.HACKRF_SUCCESS
        else:
            logger.error('Failed to set amp')

    def set_baseband_filter_bandwidth(self, bandwidth_hz):
        result = libhackrf.hackrf_set_baseband_filter_bandwidth(self.device, bandwidth_hz)
        if result != HackRfError.HACKRF_SUCCESS:
            logger.error(
                'Failed to set Baseband Filter Bandwidth with value [%d]', bandwidth_hz)
        else:
            logger.debug(
                'Successfully set Baseband Filter Bandwidth with value [%d]', bandwidth_hz)
            return HackRfError.HACKRF_SUCCESS

    def max2837_read(self, register_number, value):
        pass
        return libhackrf.hackrf_max2837_read(self.device, register_number, value)

    def max2837_weite(self, register_number, value):
        pass
        return libhackrf.hackrf_max2837_weite(self.device, register_number, value)

    def si5351c_read(self, register_number, value):
        pass
        return libhackrf.hackrf_si5351c_read(self.device, register_number, value)

    def si5351c_write(self, register_number, value):
        pass
        return libhackrf.hackrf_si5351c_write(self.device, register_number, value)

    def rffc5071_read(self, register_number, value):
        pass
        return libhackrf.hackrf_rffc5071_read(self.device, register_number, value)

    def rffc5071_write(self, register_number, value):
        pass
        return libhackrf.hackrf_rffc5071_write(self.device, register_number, value)

    def spiflash_erase(self):
        pass
        return libhackrf.hackrf_spiflash_erase(self.device)

    def spiflash_write(self, address, length, data):
        pass
        return libhackrf.hackrf_spiflash_write(self.device, address, length, data)

    def spiflash_read(self, address, length, data):
        pass
        return libhackrf.hackrf_spiflash_read(self.device, address, length, data)

    def cpld_write(self, data, total_length):
        pass
        return libhackrf.hackrf_cpld_write(self.device, data, total_length)

    def set_sample_rate_manual(self, freq_hz, divider):
        pass
        return libhackrf.hackrf_set_sample_rate_manual(self.device, freq_hz, divider)

    def compute_baseband_filter_bw_round_down_lt(self, bandwidth_hz):
        pass
        return libhackrf.hackrf_compute_baseband_filter_bw_round_down_lt(bandwidth_hz)

    def compute_baseband_filter_bw(self, bandwidth_hz):
        pass
        return libhackrf.hackrf_compute_baseband_filter_bw(bandwidth_hz)

    # def hackrf_set_freq_explicit(self, if_freq_hz, lo_freq_hz, path):
    #     pass
    # return libhackrf.hackrf_set_freq_explicit(if_freq_hz, lo_freq_hz, path)

    # def hackrf_board_partid_serialno_read(self, read_partid_serialno):
    #     pass
    # return libhackrf.hackrf_board_partid_serialno_read(read_partid_serialno)

    # def hackrf_error_name(self, errcode):
    #     pass
    #     return libhackrf.hackrf_error_name(errcode)

    # def hackrf_board_id_name(self, board_id):
    #     pass
    #     return libhackrf.hackrf_board_id_name(board_id)

    # def hackrf_filter_path_name(self, path):
    #     pass
    #     return libhackrf.hackrf_filter_path_name(path)

    # out[0] = (out[0] - 127)*(1.0/128);
    def packed_bytes_to_iq(self, bytes):
        ''' Convenience function to unpack array of bytes to Python list/array
        of complex numbers and normalize range.  size 16*32*512 262 144
        '''
        # use NumPy array
        iq = np.empty(len(bytes)//2, 'complex')
        iq.real, iq.imag = bytes[::2], bytes[1::2]
        iq /= 128.0
        return iq

    def packed_bytes_to_iq_withsize(self, bytes, size):
        ''' Convenience function to unpack array of bytes to Python list/array
        of complex numbers and normalize range.
        '''
        # use NumPy array
        iq = np.empty(size , 'complex')
        bytes2 = bytes[0:size * 2]
        iq.real, iq.imag = bytes2[::2], bytes2[1::2]
        iq /= 128.0
        return iq