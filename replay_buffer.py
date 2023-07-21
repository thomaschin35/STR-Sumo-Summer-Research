import numpy as np
import tensorflow as tf
from collections import deque

class ReplayBuffer():
    def __init__(self):
        self.replayBuffer=deque(maxlen=self.replayBufferSize)
        # number of training episodes it takes to update the target network parameters
        # that is, every updateTargetNetworkPeriod we update the target network parameters
        self.updateTargetNetworkPeriod = 100
        self.counterUpdateTargetNetwork = 0
