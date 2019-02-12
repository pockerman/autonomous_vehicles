"""
Wrap an error message to be sent to the ErrorLogger
"""

class ErrMsg(object):

    def __init__(self, type, msg):
        self.__type = type
        self.__msg = msg

    @property
    def type(self):
        return self.__type