from typing import Callable, Self, Sequence
import numpy as np
from enum import Enum

class JointsType(Enum):
    FULL = 0
    REDUCED = 1

class JointsVector:

    def __init__(self, type:JointsType, values:np.ndarray, names:list[str]|None=None):
        if names and len(names) != len(values):
            raise ValueError(f"Number of names ({len(names)}) must match number of values ({len(values)})")

        self.type = type
        self.values = values
        self.names = names
        
        self.names_to_index = {name: index for index, name in enumerate(names)} if names else None

    @classmethod
    def convert(cls, src:Self, dest:JointsType)->Self:
        conversion_fn = cls._conversion_fns[(src.type, dest)]
        return conversion_fn(src)

    def __getitem__(self, key:int|str|slice|list[str])->np.ndarray:
        if isinstance(key, int) or isinstance(key, slice):
            return self.values[key]
        elif isinstance(key, str):
            if self.names_to_index is None:
                raise ValueError(f'Trying to get "{key}", but names are not set')
            return self.values[self.names_to_index[key]]
        elif isinstance(key, list):
            if self.names_to_index is None:
                raise ValueError(f'Trying to get "{key}", but names are not set')
            return np.take(self.values, tuple(self.names_to_index[k] for k in key))
        else:
            raise ValueError(f"Invalid key type: {type(key)}")

    def __setitem__(self, key:int|str|slice|list[str], value:float|Sequence[float]):
        if isinstance(key, int) or isinstance(key, slice):
            self.values[key] = value
        elif isinstance(key, str):
            if self.names_to_index is None:
                raise ValueError(f'Trying to set "{key}", but names are not set')
            self.values[self.names_to_index[key]] = value
        elif isinstance(key, list):
            if self.names_to_index is None:
                raise ValueError(f'Trying to set "{key}", but names are not set')
            np.put(self.values, tuple(self.names_to_index[k] for k in key), value)
        else:
            raise ValueError(f"Invalid key type: {type(key)}")
    
    def __len__(self):
        return len(self.values)

    def __str__(self):
        return f"""
        {self.type} JointsVector:
        {self.values}
        """

    
class ReducedJointsVector(JointsVector):
    def __init__(self, values:np.ndarray, names:list[str]|None=None):
        super().__init__(JointsType.REDUCED, values, names)
    
    def to_full(self)->'FullJointsVector':
        raise NotImplementedError("ReducedJointsVector cannot be converted to FullJointsVector")


class FullJointsVector(JointsVector):
    def __init__(self, values:np.ndarray, names:list[str]|None=None):
        super().__init__(JointsType.FULL, values, names)

    def to_reduced(self)->'ReducedJointsVector':
        raise NotImplementedError("FullJointsVector cannot be converted to ReducedJointsVector")
