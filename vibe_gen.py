import numpy as np
import sounddevice as sd
import time

def generate_vibration(sample_rate=48000, duration=1.0, gap=0.3, amplitude=0.7):
    
    for freq in range(10, 301):
        print(f"Playing {freq} Hz")

        if freq > 90 and freq < 200:
            amplitude = 0.1
        else:
            amplitude = 0.7
            
        t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
        signal = amplitude * np.sin(2 * np.pi * freq * t)
        
        

        sd.play(signal, sample_rate)
        sd.wait()

        time.sleep(gap)
        
generate_vibration()