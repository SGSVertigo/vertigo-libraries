import pandas as pd
from tools.FileSelector import FileSelector

class FileParser:

    @staticmethod
    def selectAndOpen():
        file_path = FileSelector.select()
        data = FileParser.parse(file_path)
        data = FileParser.fixRolloverError(data)
        gpsData = FileParser.getDataType(data,1)
        imuData = FileParser.getDataType(data,2)
        quatData = FileParser.getDataType(data,3)
        return [gpsData,imuData,quatData]

    @staticmethod
    def parse(file_path):
        data = pd.read_csv(file_path)
        return data

    @staticmethod
    def getDataType(dataFrame, type_interger_identifier):
        return dataFrame.loc[dataFrame[dataFrame.columns[1]] == type_interger_identifier]

    @staticmethod
    def fixRolloverError(dataFrame):
        rollover_value = 2.0**32/1e4
        delta_times = dataFrame.diff()[dataFrame.columns[0]]
        negative_delta_times = delta_times.loc[delta_times < 0]
        times = dataFrame[dataFrame.columns[0]]
        for row in negative_delta_times.index:
            times[row:len(times.index)] += rollover_value
        dataFrame[dataFrame.columns[0]] = times
        return dataFrame
            



