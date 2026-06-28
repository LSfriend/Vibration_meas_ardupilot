from pymavlink import mavutil
import matplotlib.pyplot as plt
import numpy as np
import vibe_analysis


def plot_fft(freqs, amps, max_freq=None):
    plt.figure(figsize=(12, 5))
    plt.plot(freqs, amps)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude (m/s²)")
    plt.title("FFT Spectrum")
    plt.grid(True)

    if max_freq is not None:
        plt.xlim(0, max_freq)

    plt.show()

va = vibe_analysis.VibrationAnalyzer()

va.set_log_file("log.bin")
va.process()


   

            
# delta_t = 0 
# for i in range(1, 10000):
#     delta_t = delta_t + time_us[i] - time_us[i-1]
#     print(time_us[i] - time_us[i-1])
        

# delta_t = delta_t / 1000

# freq = 1000000 / delta_t 

# print('freq = ', freq)

# freqs, amps = compute_fft(acc_z[0:80000], freq)



# plot_fft(freqs, amps)