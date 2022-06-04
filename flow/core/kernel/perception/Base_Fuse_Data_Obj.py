from abc import ABCMeta,abstractmethod,ABC
class Base_Fuse_Data_Obj(metaclass=ABCMeta):
    @abstractmethod
    def fuse(self,*data):
        pass