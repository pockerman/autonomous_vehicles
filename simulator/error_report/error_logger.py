"""
ErrLogger logs errors produced by a system
"""

class ErrLogger(object):

    def __init__(self):
        self.__errors = dict()

    def append_error(self, err):

        if err.type in self.__errors.keys():
            self.__errors[err.type].append(err)
        else:
            self.__errors[err.type]= [err]