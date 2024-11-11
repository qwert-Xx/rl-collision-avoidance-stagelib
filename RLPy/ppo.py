import agent
import net
import torch
import os
import torch.nn as nn
import numpy as np
import time
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
class PPO:
    def __init__(self,lr = 5e-5 , gamma = 0.99 , clip = 0.1 , lam = 0.95):
        num_points = 24
        radius = 5
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        points = [[float(radius * np.cos(angle)), float(radius * np.sin(angle))] for angle in angles]
        # points = np.random.uniform(-4, 4, (num_points, 2)).tolist()
        goal = points
        print("目标点",goal)

        # goal = [ [0,0] ] * 24 #目标点
        self.agent = agent.Agent(goal,agent_num=num_points)
        self.actor = net.ActorNet().to(device)
        self.critic = net.CriticNet().to(device)
        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optim = torch.optim.Adam(self.critic.parameters(), lr=lr)
        self.gamma = gamma 
        self.clip = clip
        self.lam = lam 
        self.time_step_max_per_epoch = 600 #每局最多跑多少步
        self.epoch_per_batch = 3 #每个batch跑多少局
        self.n_updates_per_iteration = 20 #每次学习的迭代次数
        self.save_freq = 5 #保存模型的频率

        #加载模型
        #判断文件是否存在
        if os.path.exists('./ppo_actor.pth') and os.path.exists('./ppo_critic.pth'):
            self.actor.load_state_dict(torch.load('./ppo_actor.pth'))
            self.critic.load_state_dict(torch.load('./ppo_critic.pth'))
            print("加载模型成功")
        else:
            print("加载模型失败")
            #生成一个dataout.csv空文件
            with open("dataout.csv","w") as f:
                f.write("")
    def run(self):
        t = 0
        batch_state = [] #[第几局][第几个机器人][第几步]
        batch_action = []
        batch_log_prob = []
        # batch_rtgs = []
        batch_vals = []
        batch_A_k = []
        batch_rews = []


        for epoch in range(self.epoch_per_batch): #进行多少局
            
            #每局随机生成位置和目标点
            self.agent.randomSetInitPosition(radius=16,distance=5)
            self.agent.randomSetGoal(radius=16,distance=5)

            robotDons = torch.tensor( [False] * self.agent.getRobotNumber() ) #机器人是否结束
            ep_state = {"distance":[[]  for _ in range(self.agent.getRobotNumber())] ,"angle":[[]  for _ in range(self.agent.getRobotNumber())] ,"laser_data":[[]  for _ in range(self.agent.getRobotNumber()) ] ,"line_speed":[[]  for _ in range(self.agent.getRobotNumber()) ] ,"angle_speed":[[]  for _ in range(self.agent.getRobotNumber())]} #[第几个机器人][第几步]
            ep_acts = [[] for _ in range(self.agent.getRobotNumber()) ] #[第几个机器人][第几步]
            ep_log_probs = [[] for _ in range(self.agent.getRobotNumber())] #[第几个机器人][第几步]
            ep_rews = [[] for _ in range(self.agent.getRobotNumber())] #[第几个机器人][第几步]
            ep_dones = [[] for _ in range(self.agent.getRobotNumber())] #[第几个机器人][第几步]
            # ep_rtgs = [[] for _ in range(self.agent.getRobotNumber())] #[第几个机器人][第几步]
            ep_vals = [[] for _ in range(self.agent.getRobotNumber())] #[第几个机器人][第几步]
            ep_ts = [None  for _ in range(self.agent.getRobotNumber())] #[第几个机器人] 一个机器人的时间步数
            ep_A_k = None
            ep_t = 0
            state = self.agent.reset()
            successNum = 0 #成功的机器人数量
            for ep_t in range(self.time_step_max_per_epoch): #每一步
                #state = {"distance":distance,"angle":angle,"laser_data":laser_data,"line_speed":line_speed,"angle_speed":angle_speed}
                #state["distance"] #[第几个机器人] 距离目标点的距离
                for i in range(self.agent.getRobotNumber()): #每一个机器人
                    ep_state["distance"][i].append(state["distance"][i])
                    ep_state["angle"][i].append(state["angle"][i])
                    ep_state["laser_data"][i].append(state["laser_data"][i])
                    ep_state["line_speed"][i].append(state["line_speed"][i])
                    ep_state["angle_speed"][i].append(state["angle_speed"][i])

                laser_data = torch.tensor(state["laser_data"],dtype=torch.float).to(device)
                line_speed = torch.tensor(state["line_speed"],dtype=torch.float).to(device).unsqueeze(1)
                angle_speed = torch.tensor(state["angle_speed"],dtype=torch.float).to(device).unsqueeze(1)
                distance = torch.tensor(state["distance"],dtype=torch.float).to(device).unsqueeze(1)
                angle = torch.tensor(state["angle"],dtype=torch.float).to(device).unsqueeze(1)
                
                #计算action
                action_mean, action_logstd = self.actor(laser_data = laser_data, line_speed = line_speed, angle_speed = angle_speed, distance = distance, angle = angle)
                std = action_logstd.exp()
                normal = torch.distributions.MultivariateNormal(action_mean, torch.diag(std))
                action = normal.sample()
                action = torch.stack([torch.clamp(action[:,0], 0, 1), torch.clamp(action[:,1], -1, 1)] , dim = 1).to(device)
                logprob = normal.log_prob(action)
                #计算value
                value = self.critic(laser_data = laser_data, line_speed = line_speed, angle_speed = angle_speed, distance = distance, angle = angle)


                for i in range(robotDons.size()[0]):
                    if robotDons[i]: #如果机器人已经结束了
                        action[i] = torch.tensor([0,0],dtype=torch.float).to(device) #动作为0
                        logprob[i] = torch.tensor(0,dtype=torch.float).to(device)

                state,reward,done = self.agent.step({"line_speed":action[:,0].tolist(),"angle_speed":action[:,1].tolist()})   
                
                done = torch.tensor(done)

                robotDons = robotDons | done #或运算，保证结束之后始终为结束状态

                for i in range(self.agent.getRobotNumber()):
                    ep_acts[i].append(action[i])
                    ep_log_probs[i].append(logprob[i])
                    ep_rews[i].append(reward[i])
                    ep_dones[i].append(done[i])
                    ep_vals[i].append(value[i])

                for i in range(len(ep_ts)): #更新时间步数
                    if robotDons[i]: #如果机器人结束
                        if ep_ts[i] is None: #如果时间步数为空,即没有记录过
                            ep_ts[i] = ep_t + 1 #记录时间步数
                            if reward[i] > 0:
                                print("第{}个机器人在第{}步跑到了终点".format(i,ep_t))
                                successNum += 1
                            elif reward[i] < 0:
                                print("第{}个机器人在第{}步撞车了".format(i,ep_t))
                
                if robotDons.all(): #如果所有机器人都结束了
                    break


            print("成功的机器人数量:",successNum)
            #一局结束，计算一局的数据
            for i in range(len(ep_ts)):
                if ep_ts[i] is not None: #将提前结束的机器人的后面的数据删除
                    ep_state["distance"][i] = ep_state["distance"][i][:ep_ts[i]]
                    ep_state["angle"][i] = ep_state["angle"][i][:ep_ts[i]]
                    ep_state["laser_data"][i] = ep_state["laser_data"][i][:ep_ts[i]]
                    ep_state["line_speed"][i] = ep_state["line_speed"][i][:ep_ts[i]]
                    ep_state["angle_speed"][i] = ep_state["angle_speed"][i][:ep_ts[i]]
                    ep_acts[i] = ep_acts[i][:ep_ts[i]]
                    ep_log_probs[i] = ep_log_probs[i][:ep_ts[i]]
                    ep_rews[i] = ep_rews[i][:ep_ts[i]]
                    ep_dones[i] = ep_dones[i][:ep_ts[i]]
                    ep_vals[i] = ep_vals[i][:ep_ts[i]]
                else : #如果没有结束
                    ep_ts[i] = ep_t + 1



            # #计算rtgs
            # for i in range(len(ep_ts)): #对每一个机器人进行计算
            #     discounted_reward = 0
            #     for rew in reversed(ep_rews[i]): #倒序
            #         discounted_reward = rew + discounted_reward * self.gamma
            #         ep_rtgs[i].insert(0,discounted_reward)
            
            #通过GAE计算优势函数
            #TODO
            ep_A_k = self.calculate_gae(ep_rews, ep_vals, ep_dones)


            #将数据展平添加到batch中,从[第几个机器人][第几步]展平到[第几步]
            #首先将数据展平

            #先转换成tensor
            for i in range(len(ep_ts)):
                ep_state["distance"][i] = torch.tensor(ep_state["distance"][i],dtype=torch.float).to(device)
                ep_state["angle"][i] = torch.tensor(ep_state["angle"][i],dtype=torch.float).to(device)
                ep_state["laser_data"][i] = torch.tensor(ep_state["laser_data"][i],dtype=torch.float).to(device)
                ep_state["line_speed"][i] = torch.tensor(ep_state["line_speed"][i],dtype=torch.float).to(device)
                ep_state["angle_speed"][i] = torch.tensor(ep_state["angle_speed"][i],dtype=torch.float).to(device)
                ep_acts[i] = torch.stack(ep_acts[i]).to(device)
                ep_log_probs[i] = torch.stack(ep_log_probs[i]).to(device)
                ep_rews[i] = torch.tensor(ep_rews[i],dtype=torch.float).to(device)
                ep_vals[i] = torch.stack(ep_vals[i]).to(device)
                # ep_rtgs[i] = torch.tensor(ep_rtgs[i],dtype=torch.float).to(device)
            
            #展平
            ep_state["distance"] = torch.cat(ep_state["distance"],dim=0)
            ep_state["angle"] = torch.cat(ep_state["angle"],dim=0)
            ep_state["laser_data"] = torch.cat(ep_state["laser_data"],dim=0)
            ep_state["line_speed"] = torch.cat(ep_state["line_speed"],dim=0)
            ep_state["angle_speed"] = torch.cat(ep_state["angle_speed"],dim=0)
            ep_acts = torch.cat(ep_acts,dim=0)
            ep_log_probs = torch.cat(ep_log_probs,dim=0)
            ep_rews = torch.cat(ep_rews,dim=0)
            # ep_A_k = torch.cat(ep_A_k,dim=0)
            # ep_rtgs = torch.cat(ep_rtgs,dim=0)
            ep_vals = torch.cat(ep_vals,dim=0)

            #添加到batch中
            batch_state.append(ep_state)
            batch_action.append(ep_acts)
            batch_log_prob.append(ep_log_probs)
            batch_vals.append(ep_vals)
            batch_A_k.append(ep_A_k)
            batch_rews.append(ep_rews)
            # batch_rtgs.append(ep_rtgs)
        #所有局跑完
        #最后将数据展平
        state = {
            "distance":[],
            "angle":[],
            "laser_data":[],
            "line_speed":[],
            "angle_speed":[]
        }
        for i in range(len(batch_state)):
            state["distance"].append(batch_state[i]["distance"])
            state["angle"].append(batch_state[i]["angle"])
            state["laser_data"].append(batch_state[i]["laser_data"])
            state["line_speed"].append(batch_state[i]["line_speed"])
            state["angle_speed"].append(batch_state[i]["angle_speed"])
        
        batch_state = {
            "distance":torch.cat(state["distance"],dim=0),
            "angle":torch.cat(state["angle"],dim=0),
            "laser_data":torch.cat(state["laser_data"],dim=0),
            "line_speed":torch.cat(state["line_speed"],dim=0),
            "angle_speed":torch.cat(state["angle_speed"],dim=0)
        }




        batch_action = torch.cat(batch_action,dim=0)
        batch_log_prob = torch.cat(batch_log_prob,dim=0)
        batch_vals = torch.cat(batch_vals,dim=0)
        batch_A_k = torch.cat(batch_A_k,dim=0)
        batch_rews = torch.cat(batch_rews,dim=0)
        # batch_rtgs = torch.cat(batch_rtgs,dim=0)

        
        torch.cuda.empty_cache()
        return batch_state,batch_action,batch_log_prob,batch_A_k,batch_rews
                
    def learn(self,batch_state,batch_action,batch_log_prob,batch_A_k):
        V,_ = self.evaluate(batch_state,batch_action) #旧策略的价值函数
        A_k = batch_A_k

        batch_rtgs = A_k + V.detach() 

        # A_k = batch_rtgs - V.detach()
        A_k = (A_k - A_k.mean()) / (A_k.std() + 1e-10)
        for _ in range(self.n_updates_per_iteration):
            V, curr_log_probs = self.evaluate(batch_state, batch_action) #新策略的价值函数
            V = V.squeeze()
            ratios = torch.exp(curr_log_probs - batch_log_prob.detach())
            surr1 = ratios * A_k
            surr2 = torch.clamp(ratios, 1-self.clip, 1+self.clip) * A_k
            actor_loss = (-torch.min(surr1, surr2)).mean()
            critic_loss = nn.MSELoss()(V, batch_rtgs)
            print("actor_loss:",actor_loss)
            print("critic_loss:",critic_loss)

            self.actor_optim.zero_grad()
            actor_loss.backward(retain_graph=True)
            self.actor_optim.step()

            self.critic_optim.zero_grad()
            critic_loss.backward()
            self.critic_optim.step()

            if _ % self.save_freq == 0:
                torch.save(self.actor.state_dict(), './ppo_actor.pth')
                torch.save(self.critic.state_dict(), './ppo_critic.pth')
        
        torch.cuda.empty_cache()

    def evaluate(self,batch_state,batch_action):

        laser_data = batch_state["laser_data"]
        line_speed = batch_state["line_speed"].unsqueeze(1)
        angle_speed = batch_state["angle_speed"].unsqueeze(1)
        distance = batch_state["distance"].unsqueeze(1)
        angle = batch_state["angle"].unsqueeze(1)

        action_mean, action_logstd = self.actor(laser_data = laser_data, line_speed = line_speed, angle_speed = angle_speed, distance = distance, angle = angle)
        std = action_logstd.exp()
        normal = torch.distributions.MultivariateNormal(action_mean, torch.diag(std))
        action = batch_action
        logprob = normal.log_prob(action)

        V = self.critic(laser_data = laser_data, line_speed = line_speed, angle_speed = angle_speed, distance = distance, angle = angle)
        V = V.squeeze()
        return V,logprob

    def start(self):
        num = 0
        while(True):
            batch_state,batch_action,batch_log_prob,batch_A_k,batch_rews = self.run()
            print("平均奖励:",batch_rews.mean())
            print("速度概率:",self.actor.logstd.tolist())
            #将数据记录到dataout.csv中
            
            num += 1
            timestamp = time.strftime("%Y-%m-%d-%H:%M:%S")
            with open("dataout.csv","a") as f:
                #包括平均奖励,速度概率
                f.write("{},{},{}\n".format(batch_rews.mean(),self.actor.logstd.tolist(),timestamp))
                #每一百次记录一次
            if num % 100 == 0:
            #保存模型，只保存权重，文件名包含时间戳和对应于记录的第几行
                torch.save(self.actor.state_dict(), './ppo_actor_{}.pth'.format(timestamp))
                torch.save(self.critic.state_dict(), './ppo_critic_{}.pth'.format(timestamp))
            
                

            self.learn(batch_state,batch_action,batch_log_prob,batch_A_k)
            del batch_state
            del batch_action
            del batch_log_prob
            del batch_A_k
    def calculate_gae(self, rewards, values, dones):
        batch_advantages = []
        for ep_rews, ep_vals, ep_dones in zip(rewards, values, dones):
            advantages = []
            last_advantage = 0

            for t in reversed(range(len(ep_rews))):
                if t + 1 < len(ep_rews):
                    delta = ep_rews[t] + self.gamma * ep_vals[t+1] * (1 - (ep_dones[t+1] * 1)) - ep_vals[t]
                else:
                    delta = ep_rews[t] - ep_vals[t]

                advantage = delta + self.gamma * self.lam * (1 - (ep_dones[t] * 1)) * last_advantage
                last_advantage = advantage
                advantages.insert(0, advantage)

            batch_advantages.extend(advantages)

        return torch.tensor(batch_advantages, dtype=torch.float).to(device) 

            

            




        



    