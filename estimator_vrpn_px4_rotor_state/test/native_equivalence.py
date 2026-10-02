#!/usr/bin/env python3
"""Replay the former runtime fixture through the owner core and installed ABI."""
import math
import struct
import subprocess
import sys

samples = []
for i in range(1, 1601):
    t = 1.0 + i * 0.005
    r, w = 0.5, 0.8
    samples.append((t, 0, [-r*w*w*math.sin(w*t), -r*w*w*math.cos(w*t),
                            -0.2*0.09*math.sin(0.3*t) + 9.8066, 0.0, 0.0, 0.0]))
for i in range(1, 801):
    t = 1.0 + i * 0.01 + 0.001
    if 5.0 <= t < 5.3:
        continue  # exercise Coasting and recovery
    samples.append((t, 1, [0.5*math.sin(0.8*t), 0.5*math.cos(0.8*t)-0.5,
                            1.0+0.2*math.sin(0.3*t), 1.0, 0.0, 0.0, 0.0]))
samples.sort(key=lambda s: (s[0], s[1]))
def bits(value):
    return format(struct.unpack('=Q', struct.pack('=d', value))[0], '016x')
fixture = ''.join(str(port) + ' ' + ' '.join(map(bits, [t] + values)) + '\n'
                  for t, port, values in samples).encode()
reference = subprocess.run([sys.argv[1]], input=fixture, check=True, stdout=subprocess.PIPE).stdout
native = subprocess.run([sys.argv[2]], input=fixture, check=True, stdout=subprocess.PIPE).stdout
assert reference == native, 'native state/vision event bytes differ from owner core replay'
lines = native.splitlines()
counts = {kind: sum(line.startswith(kind) for line in lines) for kind in [b'S ', b'V ']}
assert counts[b'S '] > 500 and counts[b'V '] > 100, counts
print('Exact core/native replay:', len(samples), 'input samples;', counts, 'output events')
