class SubTask(object):
    def __init__(self, name, func, owner):
        self._name = name
        self._func = func
        self._owner = owner

    def run(self, **kwargs):
        return self._func(**kwargs)

    # @property
    # def name(self):
    #     return self._name
    # @name.setter
    # def name(self, v):
    #     self._name = v        

    # @property
    # def func(self):
    #     return self._func
    # @func.setter
    # def func(self, v):
    #     self._func = v        

