from abc import ABC, abstractmethod

class MetricBase(ABC):

    @abstractmethod
    def evaluate(self, trajectory, map_data=None,agent_data=None):
        pass