import pylibhackrf ,ctypes

hackrf = pylibhackrf.HackRf()

if hackrf.is_open == False:
    hackrf.setup()
    hackrf.set_freq(100 * 1000 * 1000)
    hackrf.set_sample_rate(8 * 1000 * 1000)
    hackrf.set_amp_enable(False)
    hackrf.set_lna_gain(16)
    hackrf.set_vga_gain(20)    

def callback_fun(hackrf_transfer):
    array_type = (ctypes.c_byte*length)
    values = ctypes.cast(hackrf_transfer.contents.buffer, ctypes.POINTER(array_type)).contents
    #iq data here
    iq = hackrf.packed_bytes_to_iq(values)    
    return 0

hackrf.start_rx_mode(callback_fun)