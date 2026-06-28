from pymavlink import mavutil
import matplotlib.pyplot as plt
import numpy as np
import imu_data

class VibrationAnalyzer:
    def __init__(self):
        self.log_file = None
        self.log = []
        self.processed = False
        self.IMUs = [imu_data.IMUData(), imu_data.IMUData(), imu_data.IMUData()]

    def set_log_file(self, log_file):
        """
        Set ArduPilot .bin log file.

        Parameters:
            log_file (str): path to .bin file
        """
        self.log_file = log_file

    def process(self):
        """
        Parse and process all log data.

        Expected responsibilities:
        - load .bin file
        - extract IMU data
        - split into frequency windows
        - compute FFT / vibration amplitudes
        - compute attenuation
        """
        self.log = mavutil.mavlink_connection(self.log_file)
        
        self.parse_all_IMU()
        
        self.get_vibration_on_freq(20 * 1000000, 30 * 1000000, 0, 200.0)
        
        
        
    def parse_all_IMU(self):
        if self.log == []:
            exit

        while True:
            msg = self.log.recv_match(blocking=False)
            if msg is None:
                break
            
            if msg.get_type() == "ACC":
                self.IMUs[msg.I].time.append(msg.TimeUS)
                self.IMUs[msg.I].ax.append(msg.AccX)
                self.IMUs[msg.I].ay.append(msg.AccY)
                self.IMUs[msg.I].az.append(msg.AccZ)
                
        print(f"parser: Loaded {len(self.IMUs[0].time)} samples")
        
        """ Calculate IMU frequency """
        frequency = estimate_sample_rate(self.IMUs[0].time)
        
        for i in range(0, len(self.IMUs)):
            self.IMUs[i].freq = frequency
            


    def get_attenuation_on_freq(self):
        """
        Return attenuation vs frequency.

        Returns:
            tuple:
                frequencies (list[float])
                attenuations_db (list[float])
        """
        

    
    def get_vibration_on_freq(self, timeStart, timeEnd, imuNum, freq, axis = 'Z'):
        
        startIndex = 0
        endIndex = 0
        while self.IMUs[imuNum].time[startIndex] < timeStart:
            startIndex += 1
            
        while self.IMUs[imuNum].time[startIndex + endIndex] < timeEnd:
            endIndex += 1
        
        freqs = []
        apms = []
        if axis == 'Z':
            freqs, amps = compute_fft(self.IMUs[imuNum].az[startIndex:endIndex], self.IMUs[imuNum].freq)
        elif axis == 'X':
            freqs, amps = compute_fft(self.IMUs[imuNum].ax[startIndex:endIndex], self.IMUs[imuNum].freq)
        elif axis == 'Y':
            freqs, amps = compute_fft(self.IMUs[imuNum].ay[startIndex:endIndex], self.IMUs[imuNum].freq)
            
        return get_peak_near_frequency(freqs, amps, freq)
        
        

def get_peak_near_frequency(freqs, amps, target_freq, bandwidth=2):
    mask = np.abs(freqs - target_freq) <= bandwidth

    local_freqs = freqs[mask]
    local_amps = amps[mask]

    peak_idx = np.argmax(local_amps)

    #return local_freqs[peak_idx], local_amps[peak_idx]
    return local_amps[peak_idx]

def compute_fft(signal, sample_rate):
    """
    signal: 1D numpy array
    sample_rate: Hz

    returns:
        freqs  - frequency bins
        amps   - amplitude spectrum
    """

    signal = np.asarray(signal, dtype=np.float64)

    # remove DC
    signal = signal - np.mean(signal)

    # largest power of 2 <= len(signal)
    n = 2 ** int(np.floor(np.log2(len(signal))))
    signal = signal[:n]

    # Hann window
    window = np.hanning(n)
    signal_windowed = signal * window

    # FFT
    fft = np.fft.rfft(signal_windowed)
    freqs = np.fft.rfftfreq(n, d=1/sample_rate)

    # amplitude normalization
    amps = np.abs(fft) * 2 / np.sum(window)

    return freqs, amps

def estimate_sample_rate(time_us):
    time_us = np.array(time_us)

    dt_us = np.diff(time_us)
    avg_dt_us = np.mean(dt_us)

    fs = 1e6 / avg_dt_us
    return fs