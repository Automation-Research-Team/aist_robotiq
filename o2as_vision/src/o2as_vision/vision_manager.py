import rospy
from vision_group_interface import VisionGroupInterface

class VisionManager(object):
    def __init__(self):
        self._items = dict()
    
    def prepare(self):
        for key in self._items:
            self._items[key].prepare()

    def update_scene(self):
        for key in self._items:
            self._items[key].find_object()

    def add_group(self, name):
        self._items[name] = VisionGroupInterface(name)
        return self._items[name]

    def get_group(self, name):
        return self._items[name]
