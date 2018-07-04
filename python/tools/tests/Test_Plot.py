import unittest
from tools.FileParser import FileParser
import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt

class Test_Plot(unittest.TestCase):

    def test_Plot(self):
        df = FileParser.selectAndOpen()
        df[1].plot(df[1].columns[0],df[1].columns[2])
        plt.show()

if __name__ == '__main__':
    unittest.main()