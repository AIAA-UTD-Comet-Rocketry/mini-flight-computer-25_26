from scipy.signal import butter
import numpy as np
from scipy.signal import freqz
import matplotlib.pyplot as plt

b, a = butter(N=2, Wn=5.0, btype='low', fs=100.0)

print("b (feedforward):", b)   # [b0, b1, b2]
print("a (feedback):   ", a)   # [1.0, a1, a2]

# esp-dsp coef array is: [b0, b1, b2, a1, a2]
# Take a[1] and a[2] directly from scipy's output
coef = [b[0], b[1], b[2], a[1], a[2]]
print("esp-dsp coef:", coef)

# Verify stability: all poles must be inside the unit circle
roots = np.roots(a)
print("Poles:", roots)
print("|poles|:", np.abs(roots))   # must all be < 1.0

w, h = freqz(b, a, fs=100)
plt.plot(w, 20*np.log10(np.abs(h)))
plt.axvline(5, color='r', label='fc=5Hz')
plt.xlabel('Hz'); plt.ylabel('dB'); plt.grid(); plt.legend(); plt.show()
