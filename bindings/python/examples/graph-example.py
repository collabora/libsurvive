import pysurvive
import pysurvive.recorder

import matplotlib.pyplot as plt

import sys

ctx = pysurvive.init(sys.argv)

if ctx is None: # implies -help or similiar
    exit(-1)

recorder = pysurvive.recorder.install(ctx)

while pysurvive.poll(ctx) == 0:
    pass

recorder.plot()
plt.show()

pysurvive.close(ctx)
