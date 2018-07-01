import tkinter as tk
from tkinter import filedialog

class File-Selector:

    @staticmethod
    def select():
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename()
        return file_path

    @staticmethod
    def selectAndOpen():
        file_path = select()