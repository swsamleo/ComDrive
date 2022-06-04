from abc import ABCMeta,abstractmethod,ABC
class Base_Error_Obj(metaclass=ABCMeta):
    def __init__(self,error_size):
        self.error_size = error_size

    @abstractmethod
    def generate_error(self):
        pass