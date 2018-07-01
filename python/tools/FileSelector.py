import pandas
import tkinter as tk
from tkinter import filedialog


class FileSelector:

    @staticmethod
    def select():
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(filetypes = (("Vertigo Datalogger files", "*.csv"),("All files", "*.*") ))
        return file_path
