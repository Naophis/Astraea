import numpy as np
import matplotlib.pyplot as plt

from slalom import Slalom
from plot import Plot
from plotorval import PlotOrval

import sys

p = Plot()
po = PlotOrval()

v = 1000
dia45_mode = 0
show = True
# show = False
if len(sys.argv) > 1:
    v = int(sys.argv[1])
    dia45_mode = int(sys.argv[2])
    show = False

K = 100
list_K_y = [0.5]
# K = 13540
# show = False

offset = {
    "prev": 7,
    "after": 0,  # not use
    "prev_dia": 7,
    "after_dia": 0,  # not use
}

# p.exe("normal", v, show, 0, K, list_K_y, offset)
p.exe("large", v, show, 0, K, list_K_y, offset)
# p.exe("orval", v, show, 0, K, list_K_y, offset)
# p.exe("dia45", v, show, 0, K, list_K_y, offset)
# p.exe("dia45", v, show, dia45_mode, K, list_K_y, offset)
# p.exe("dia45_2", v, show, 0, K, list_K_y, offset)
# p.exe("dia135", v, show, 0,  K, list_K_y, offset)
# p.exe("dia135_2", v, show, 0,  K, list_K_y, offset)
# p.exe("dia90", v, show, 0, K, list_K_y, offset)
