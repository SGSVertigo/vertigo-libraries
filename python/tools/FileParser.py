import pandas as pd

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
        return dataFrame.loc[dataFrame[1] == type_interger_identifier]

    @staticmethod
    def fixRolloverError(dataFrame):
        rollover_value = 2^32/1e4
        delta_times = dataFrame.iloc[:,[0]].diff()
        negative_delta_times = delta_times.loc[delta_times[0] < 0]
        for row in negative_delta_times:
            dataFrame = dataFrame[:,row.index():-1] + rollover_value
        return dataFrame
            



