import numpy as np
import matplotlib.pyplot as plt
import seaborn as sb
import math


class Graph(object):
    def __init__(self):
        self.points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
        


    def update_heatmap(data):

