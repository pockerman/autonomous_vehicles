
class ControllerBase(object):

    def __init__(self):
        self.__properties = dict()


    def get_property(self, name):
        return self.__properties[name]

    def set_property(self, name, value):
        self.__properties[name] = value

    def has_property(self, name):
        return name in self.__properties.keys()