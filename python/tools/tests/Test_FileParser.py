import unittest
from tools import FileParser
import pandas as pd

class Test_FileParser(unittest.TestCase):

    def test_getDataType(self):
        d = {'time': [0,0,1,1,2,2,3,3,4,4], 'type': [1,2,1,2,1,2,1,2,1,2]}
        frame = pd.DataFrame(d)
        dataSeries = FileParser.getDataType(frame,1)
        self.assertEqual(5, len(dataSeries.index))

    def test_fixRolloverError(self):
        rollover_value = 2.0**32/1e4
        d = {'time': [0,1,2,0,1,2,3,0], 'type': [1,1,1,1,1,1,1,1]}
        frame = pd.DataFrame(d)
        dataSeries = FileParser.fixRolloverError(frame)
        self.assertEqual(0, dataSeries.at[0,dataSeries.columns[0]])
        self.assertEqual(1, dataSeries.at[1,dataSeries.columns[0]])
        self.assertEqual(2, dataSeries.at[2,dataSeries.columns[0]])
        self.assertEqual(rollover_value, dataSeries.at[3,dataSeries.columns[0]])
        self.assertEqual(rollover_value+1, dataSeries.at[4,dataSeries.columns[0]])
        self.assertEqual(rollover_value+2, dataSeries.at[5,dataSeries.columns[0]])
        self.assertEqual(rollover_value+3, dataSeries.at[6,dataSeries.columns[0]])
        self.assertEqual(rollover_value*2, dataSeries.at[7,dataSeries.columns[0]])

if __name__ == '__main__':
    unittest.main()