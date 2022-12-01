import numpy as np
import torch

from ..objective import Objective

class EqualMaxima(Objective):
    def __init__(
        self,
        xs_to_scores_dict={},
        num_calls=0,
        tau=None,
        **kwargs,
    ):
        super().__init__(
            xs_to_scores_dict=xs_to_scores_dict,
            num_calls=num_calls,
            task_id='test',
            dim=1,
            lb=0.0,
            ub=1.0,
            **kwargs,
        )

    def query_oracle(self, x):
        # x is torch.tensor
        print("query oracle")
        print(x)
        print(type(x))
        
        if torch.is_tensor(x):
            x = x.detach().cpu().numpy()
            print(x)
            print(x.shape)
        
        return self._evaluate(x)

    def divf(self, x1, x2 ):
        print("divf")
        print(x1)
        print(type(x1))
        return None
    
    def _evaluate(self, x: np.ndarray):
        if x is None:
            return None

        return np.sin(5.0 * np.pi * x) ** 6

class Himmelblau(Objective):
    def __init__(
        self,
        xs_to_scores_dict={},
        num_calls=0,
        tau=None,
        **kwargs,
    ):
        super().__init__(
            xs_to_scores_dict=xs_to_scores_dict,
            num_calls=num_calls,
            task_id='test',
            dim=2,
            lb=-6.0,
            ub=6.0,
            **kwargs,
        )

    def query_oracle(self, x: torch.tensor):
        # x is torch.tensor of size [ndim], # in np is size (ndim, )
        # return float

        # print("query oracle")
        # print(x)
        # print(type(x))
        
        if torch.is_tensor(x):
            x = x.detach().cpu().numpy()
            # print(x)
            # print(x.shape)
        
        return self._evaluate(x)

    def divf(self, x1: torch.tensor, x2: torch.tensor):
        # x1, x2 is torch.tensor of size [ndim]
        # in np is size (ndim, )
        # return float
        
        # print("divf")
        # print(x1, x2, x1.size())
        
        if torch.is_tensor(x1) or torch.is_tensor(x2):
            x1 = x1.detach().cpu().numpy()
            x2 = x2.detach().cpu().numpy()

        return np.linalg.norm(x1 - x2)
    
    def _evaluate(self, x: np.ndarray):
        if x is None:
            return None

        result = 200 - (x[0] ** 2 + x[1] - 11) ** 2 - (x[0] + x[1] ** 2 - 7) ** 2
        return result


