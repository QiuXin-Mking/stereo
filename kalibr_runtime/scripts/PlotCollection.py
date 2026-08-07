"""Headless replacement for Kalibr's optional wx plot window."""

from collections import OrderedDict


class PlotCollection:
    def __init__(self, window_name="", window_size=(800, 600)):
        self.frame_name = window_name
        self.window_size = window_size
        self.figureList = OrderedDict()

    def add_figure(self, tabname, figure):
        self.figureList[tabname] = figure

    def delete_figure(self, name):
        self.figureList.pop(name, None)

    def show(self):
        return None
