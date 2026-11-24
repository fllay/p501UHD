import sys, numpy as np, uhd
fs = 30.72e6; fc = 3489.42e6
u = uhd.usrp.MultiUSRP("type=p501")
def psd(x, nfft=2048):
    n = len(x)//nfft
    X = np.fft.fftshift(np.fft.fft(x[:n*nfft].reshape(n, nfft) * np.hanning(nfft), axis=1), axes=1)
    return 10*np.log10(np.mean(np.abs(X)**2, axis=0)/ (nfft*np.sum(np.hanning(nfft)**2)/nfft) + 1e-20)
f = np.fft.fftshift(np.fft.fftfreq(2048, 1/fs))/1e6
for g in [float(a) for a in sys.argv[1:]]:
    s = u.recv_num_samps(int(fs*0.25), fc, fs, [0], g)
    x = s[0].astype(np.complex64)
    p = psd(x[int(fs*0.02):])
    tot = 10*np.log10(np.mean(np.abs(x[int(fs*0.02):])**2)+1e-20)
    dc = 10*np.log10(np.abs(np.mean(x))**2+1e-20)
    def band(a, b): m = (f>=a)&(f<b); return np.mean(p[m])
    med = np.median(p); k = np.argmax(p)
    print(f"gain={g:4.0f}  total={tot:6.1f} dBFS  DCmean={dc:6.1f}  clip={np.mean(np.abs(x.real)>0.98)+np.mean(np.abs(x.imag)>0.98):.4f}")
    print("   PSD dB/bin  edge[-15,-13.5]=%.1f  [-13.5,-10]=%.1f  [-10,-5]=%.1f  [-5,-1]=%.1f  DC[-0.1,0.1]=%.1f  [1,5]=%.1f  [5,10]=%.1f  [10,13.5]=%.1f  edge[13.5,15]=%.1f" % (band(-15,-13.5), band(-13.5,-10), band(-10,-5), band(-5,-1), band(-0.1,0.1), band(1,5), band(5,10), band(10,13.5), band(13.5,15)))
    print("   median=%.1f  peak=%.1f at %.2f MHz  (peak-median=%.1f dB)" % (med, p[k], f[k], p[k]-med))
