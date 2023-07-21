import numpy as np
import tensorflow as tf
from tensorflow import keras
from collections import deque

from numba import jit, cuda
import random

class Agent:
    def __init__(self, state_size, action_size, number_of_vehicles):
        self.gamma = 0.9
        self.epsilon = 1.0
        self.epsilon_decay = 0.99995
        self.epsilon_min = 0.01
        self.learning_rate = 0.001
        
        self.replayBufferSize = 300
        self.batchReplayBufferSize = 100
        self.replayBuffer=deque(maxlen=self.replayBufferSize)
        # number of training episodes it takes to update the target network parameters
        # that is, every updateTargetNetworkPeriod we update the target network parameters
        self.updateTargetNetworkPeriod = 100
        self.counterUpdateTargetNetwork = 0

        self.number_of_vehicles = number_of_vehicles #change with experiment
        
        self.state_size = state_size
        self.action_size = action_size

        self.main_model = self.build_model()
        self.target_model = self.build_model()
        self.target_model.set_weights(self.main_model.get_weights())

        print(self.main_model.summary())


    def build_model(self):
        main_model = keras.models.Sequential(name = "sequential_1")
        # print(self.state_size)
        main_model.add(keras.layers.Dense(64, input_shape=(self.state_size, ), name="dense_1"))
        #hidden layers
        main_model.add(keras.layers.BatchNormalization())
        main_model.add(keras.layers.Dense(700, activation='relu', name="dense_2"))
        main_model.add(keras.layers.LeakyReLU(700))
        main_model.add(keras.layers.Dense(700, activation='relu', name="dense_3"))
        main_model.add(keras.layers.LeakyReLU(700))
        main_model.add(keras.layers.Dense(700,activation='relu', name="dense_4"))
        #output
        main_model.add(keras.layers.Dense(6, activation='softmax', name="dense_5")) 
        opt = keras.optimizers.RMSprop(learning_rate=self.learning_rate, momentum=0.9)
        main_model.compile(optimizer=opt, loss='sparse_categorical_crossentropy', metrics=['accuracy'])
        return main_model
    # @jit(target_backend='cuda')
    def act(self, states, eps):
        
        if (states):
            index = [np.arange(3,9)]
            states_temp = np.array(list(states.values()), dtype=np.float32)
            state_vals = states_temp[: , index ] # [1,0,1,1,0,0]
            #EPSILON EXPLORATION APPROACH
            if eps < 4 or np.random.rand() <= self.epsilon: 
                arr = self.generate_random_array(len(states))#2D arrow of vehicles, and random decision
                state_vals = state_vals.reshape(arr.shape)
                arr = arr - 10000 * (1-state_vals)
                return arr
            # GREEDY ACTIONS
            # print("actually predicting")
            act_values = self.main_model.predict(states_temp, verbose=0)
            state_vals = state_vals.reshape(act_values.shape)
            mod_values = act_values - 10000 * (1 - state_vals)
            #act values   
            #(#vehicles, 6 columns)
            # return np.argmax(act_values[0])
            return mod_values
        else:
            return {}
    # @jit(target_backend='cuda')
    def train(self,episode):
        #State/nextstate = Dict, {vid: "state"}
        #action = 2d array (#vehicles, 6 columns. probability that they pick the direction)
        #reward =  Dict {vid: "reward"}
        #done boolean flag 
        # print(state)
        # If replay buffer is full, then train the main_model
        if (len(self.replayBuffer) > self.batchReplayBufferSize) : 

            # print("training!")
            randomSampleBatch = random.sample(self.replayBuffer, self.batchReplayBufferSize)
            currentStateBatch = [np.zeros(self.state_size)]
            #(index: state values)
            nextStateBatch = [np.zeros(self.state_size)]

            for index, (state, action, reward, next_state, done) in enumerate(randomSampleBatch):

                #updating reward size and next_state size
                keys_to_remove = [key for key in reward if key not in state]
                for key in keys_to_remove:
                    reward.pop(key)

                keys_to_remove = [key for key in next_state if key not in reward]
                for key in keys_to_remove:
                    next_state.pop(key)
                currentStateBatch = np.append(currentStateBatch, np.array(list(state.values()), dtype=np.float32), axis=0)
                nextStateBatch = np.append(nextStateBatch, np.array(list(next_state.values()), dtype=np.float32), axis=0)

            currentStateBatch = currentStateBatch[1:]
            nextStateBatch = nextStateBatch[1:]

            QcurrentStateMainModel = self.main_model.predict(currentStateBatch, verbose=0)
            QnextStateTargetModel = self.target_model.predict(nextStateBatch, verbose=0)

            inputNetwork = currentStateBatch
            outputNetwork = []
            i = 0
            k = 0
            #REWARDS AND STATE AND ACTION HAVE SAME DIMENSIONS
            for index, (state, action, reward, next_state, done) in enumerate(randomSampleBatch):
                predicted_next = QnextStateTargetModel[i: i+len(list(next_state.values()))]
                predicted_current = QcurrentStateMainModel[k: k+len(list(state.values()))]
                if done:
                    target = list(reward.values())
                else:
                    target = (list(reward.values()) + self.gamma * np.amax(predicted_next, axis=1))
                for j in range(len(action)):
                    action_index = np.argmax(action[j]) #getting the index(direction) with the highest number (the action it just performed)
                    predicted_current[j, action_index] = target[j] #for each vehicle, assigning the target value to the highest number index- basically making the q value lower 
                max_q_values_i = np.argmax(predicted_current, axis=1)
                outputNetwork = np.append(outputNetwork, max_q_values_i)
                    #1D list of Target Q values that represent each row in sequence
                i += len(list(next_state.values()))
                k += len(list(state.values()))
            # print("input netowrk", inputNetwork)
            # print("output netowrk", outputNetwork)
            self.main_model.fit(inputNetwork, outputNetwork, batch_size=self.batchReplayBufferSize, verbose = 0, epochs = 50)
            self.counterUpdateTargetNetwork += 1
            if (self.counterUpdateTargetNetwork > (self.updateTargetNetworkPeriod - 1)):
                #copt the weights of main to target network
                self.target_model.set_weights(self.main_model.get_weights())
                print("target network has been updated")
                print("Counter value {}".format(self.counterUpdateTargetNetwork))
                #reset the counter
                self.counterUpdateTargetNetwork = 0
            
            if len(self.replayBuffer) > 200:
                for _ in range(2):
                    self.replayBuffer.popleft()

            if episode > 100 and self.epsilon > self.epsilon_min:
                self.epsilon *= self.epsilon_decay
            # WITHOUT REPLAY
            # # print(" ")
            # # print("state", state)
            # # print("reward", reward)
            # # print("next_state before", next_state)
            # reward_cp = reward.copy()
            # next_state_cp = next_state.copy()
            # keys_to_remove = [key for key in reward if key not in state]
            # for key in keys_to_remove:
            #     reward_cp.pop(key)
            # keys_to_remove = [key for key in next_state if key not in reward_cp]
            # for key in keys_to_remove:
            #     next_state_cp.pop(key)
            # # print("reward_copy", reward_cp)
            # # print("next_state", next_state_cp)
            # target = list(reward_cp.values()) #rewards is a 1D array of (vehicles) and rewards
            # next_states_arr = np.array(list(next_state_cp.values()), dtype=np.float32)
            # # print(next_states_arr)
            # predicted_next = self.main_model.predict(next_states_arr, verbose=0) #
            # # print("predict results: ")
            # # print(predicted_next)
            # # print("1d array of maximum q values for each vehicle")
            # # print(np.amax(predicted, axis=1)) # grabs the maximum q-value 
            # if not done:
            #     target = (list(reward_cp.values()) + self.gamma * np.amax(predicted_next, axis=1)) #1d array (vehicles, )
            # state_arr = np.array(list(state.values()), dtype=np.float32)
            # # print("state-current", state_arr)
            # predicted_current = self.main_model.predict(state_arr, verbose = 0)
            # #in predicted_current
            # # print("action: ", action)
            # # print(predicted_current)
            # # print(target)
            # for i in range(len(action)):
            #     action_index = np.argmax(action[i]) #getting the index(direction) with the highest number (the action it just performed)
            #     predicted_current[i, action_index] = target[i] #for each vehicle, assigning the target value to the highest number index- basically making the q value lower 
            # max_q_values_i = np.argmax(predicted_current, axis=1) #selecting the maximum value's index and putting it in the fit main_model
            # # print(max_q_values_i)
            # #max = 1D array (index=vehicle, value=direction that has highest probability)
            # # predicted_current = 2D array
            #     # [rows = vehicles, columns = probabilities for each direction]
            # #[0, 1, 2, 2, 1, ,1, 5, 1, 2, 4, 0 ...]
            # self.main_model.fit(state_arr, max_q_values_i, epochs=1, verbose=0)

    def generate_random_array(self, num_rows):
        arr = np.zeros((num_rows, 6))  # Initialize the array with zeros
        col_indices = np.random.randint(0, 6, size=num_rows)  # Generate random column indices

        for i in range(num_rows):
            arr[i, col_indices[i]] = 1  # Set the value of 1 in the randomly selected column
        return arr

    

