from abc import ABCMeta,abstractmethod,ABC
class Error_obj(metaclass=ABCMeta):
    def __init__(self,error_size):
        self.error_size = error_size

    @abstractmethod
    def generate_error(self):
        pass