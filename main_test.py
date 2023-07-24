import numpy as np
import agent as ag
import sumo_env as se
import matplotlib.pyplot as plt
import csv

env_train = se.SumoEnv(10, gui = False)
env_test = se.SumoEnv(10, gui = False)

max_episodes = 750
max_steps = 350
num_actions = 6
num_vehicles = 10

state_shape = len(env_train.connection_info.edge_list) + 9
agent = ag.Agent(state_shape, num_actions, num_vehicles)
rewards_history = []
for episode in range(max_episodes):
    #TRAINING LOOP
    print("epispde: " + str(episode))
    states = env_train.reset() # 2d array [rows = vehicles states] columns = state variables
    rewards = {str(key): 0 for key in range(num_vehicles)}
    overall_reward = {}
    for step in range(max_steps):
        #select Action
        # print("states ", states)
       
        actions = agent.act(states, episode)
        # print("actions",actions)
        #take a step and return state, reward, done
        next_states, rewards, done, to_direct_vehicles, arrived = env_train.step_d(actions, rewards, states)
        #adding to replay buffer - tuple= (S, A, R, S_, T)
        # print(type(rewards))
        # print(rewards)
        if len(states) != 0 and len(next_states) != 0:
            # print("before adding to buffer")
            # print(states)
            # print(next_states)
            agent.replayBuffer.append((states.copy(), actions, rewards.copy(), next_states.copy(), done)) 
            if (arrived):
                agent.arrivedBuffer.append((states.copy(), actions, rewards.copy(), next_states.copy(), done))

        agent.train(episode)
        # print("trained")
        # print("arrived list: ", arrived_list)

        for vid in states:
            if vid not in to_direct_vehicles:
                next_states.pop(vid)
        states = next_states
        if done:
            break
    env_train.close()
    overall_reward = rewards
    #TESTING LOOP
    # states = env_test.reset()
    # done = False
    # rewards = []
    # for step in range(max_steps):
    #     actions = agent.act(states)
    #     next_states, rewards, done = env_test.step_d(actions, rewards)
    #     states = next_states
    #     if done:
    #         break
    # env_test.close()  
    #grabbing awards for stats
    overall_reward = np.array(list(overall_reward.values()))
    avg_reward = np.mean(overall_reward)
    rewards_history.append(avg_reward)
    print("Episode:", episode)
    print("Rewards: ", rewards)
    print("Average reward per episode:", avg_reward)
    print("Epsilon: " + str(agent.epsilon))
eps = np.arange(0, max_episodes)
plt.plot(eps, rewards_history)
plt.xlabel("Episode")
plt.ylabel("Average Reward")
plt.title("Reward Progression")
plt.show()

headers = ['Episode', 'Average Reward'] #11

with open('./Data/qlearning_training_results3.csv', 'w', newline='') as f:
    write = csv.writer(f)
    write.writerow(headers)
    for episode, rew in zip(eps, rewards_history):
        write.writerow([episode, rew])
    f.flush()
print("data written to CSV file")
agent.main_model.save("trained_model_3.h5")
agent.target_model.save("target_model_3.h5")




        

            