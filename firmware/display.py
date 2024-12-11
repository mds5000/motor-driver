from serial import Serial
from struct import Struct
from itertools import batched
import pylab as plt
import time
import math

ZTELEM = Struct(">xIffffff")
PTELEM = Struct(">xIfffffi")
SPD = Struct(">cfc")
POS = Struct(">cic")

s = Serial("COM20", baudrate=921600, timeout=0.1)
s.write(b"e\x01e")
#time.sleep(2)

cnt = 0
o = []
p = []
i = []
d = []
rpm = []
po = []
pp = []
pi = []
pd = []
pt = []
pos = []
cmd = []

start = time.time()

rem = b""
while cnt < 2000:
    lines = list(map(bytes, batched(rem + s.read(4096), 32)))
    if len(lines[-1]) < 31:
        print("EOL", lines[-1])
        rem = lines[-1]
    
    for line in lines:
        if line[0] == 122:
            print(ZTELEM.unpack_from(line))
        if line[0] == 112:
            print(PTELEM.unpack_from(line))

    if len(line) == (1+4*11):
        (t, i, r, c, v, w, x, y, z, p, u) = FORMAT.unpack_from(line[1:])
        #print(dcp, isnsp, isetp, rpmp)

        time.append(t)  # isns
        icmd.append(i)  # icmd
        rpm.append(r)  # rpm
        rpm_c.append(c)  # rpmcmd
        rp.append(v) # Speed P
        ri.append(w) # Speed I
        rd.append(x) # Speed D
        pp.append(y) # Pos P
        pi.append(z) # Pos I
        pos.append(p) # Position
        targ.append(u) # Position Target

    if False:
        if cnt == 1000:
            s.write(POS.pack(b'p', 15000, b'p'))
        if cnt == 3000:
            s.write(POS.pack(b'p', 0, b'p'))
        if cnt == 5000:
            s.write(POS.pack(b'p', -15000, b'p'))
        if cnt == 7000:
            s.write(POS.pack(b'p', 0, b'p'))


    if True:
        freq = 1.00
        c = 10000 * math.sin((time.time()-start) * 2*math.pi * freq) 
        c = int(c)
        if cnt % 20 == 0:
            s.write(POS.pack(b'p', c, b'p'))
        
    cnt += 1



plt.figure()
plt.subplot(311)
plt.plot(o)
plt.plot(p)
plt.plot(i)
plt.plot(d)
plt.legend(["torque", "p", "i", "d"])
plt.subplot(312)
plt.plot(rpm)
plt.plot(po)
plt.plot(pp)
plt.plot(pi)
plt.plot(pd)
plt.legend(["RPM", "CMD", "p", "i", "d"])
plt.subplot(313)
plt.plot(pos)
plt.plot(pt)
plt.legend(["pos", "command"])
plt.show()