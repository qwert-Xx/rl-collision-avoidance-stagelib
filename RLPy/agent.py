import stgCPPToPy
import math

REWARD_ARRIVE = 15  #到达目标点时的奖励
REWARD_WG = 2.5  #向目标点靠近的奖励系数

REWARD_COLLISION = -15  #碰撞的惩罚

REWARD_WW = -0.1  #角速度过大的惩罚系数

class Agent():
    def __init__(self,goal,num_world = 1) -> None:
        if num_world != 1:
            raise ValueError("暂未实现多于1个世界")
        stgCPPToPy.Start(num_world)
        self.goal = goal #目标点,这是一个二维列表[robots][x,y]
        self.last_distance = None

    def sendCmd(self,cmd):#发送命令，即发送命令，返回执行的结果
        return stgCPPToPy.pycall(cmd)
    def getRobotNumber(self):
        #暂未实现 TODO
        return 24
    def reset(self):
        Cmds = []
        cmd = stgCPPToPy.RobotCmd()
        worldId = 0
        robotId = list(range(self.getRobotNumber()))
        vx = [0.0 for i in range(self.getRobotNumber())]
        vy = [0.0 for i in range(self.getRobotNumber())]
        vtheta = [0.0 for i in range(self.getRobotNumber())]
        reset = [True for i in range(self.getRobotNumber())]
        cmd.id = worldId
        cmd.robotId = robotId
        cmd.vx = vx
        cmd.vy = vy
        cmd.vtheta = vtheta
        cmd.reset = reset
        Cmds.append(cmd)
        worldData =  self.sendCmd(Cmds)[0]
        self.last_distance = None


    def step(self ,action):
        #暂未实现 TODO

        vx = action["line_speed"]
        vtheta = action["angle_speed"]
        cmds = []
        cmd = stgCPPToPy.RobotCmd()
        worldId = 0
        robotId = list(range(self.getRobotNumber()))
        vy = [0.0 for i in range(self.getRobotNumber())]
        reset = [False for i in range(self.getRobotNumber())]
        cmd.id = worldId
        cmd.robotId = robotId
        cmd.vx = vx
        cmd.vy = vy
        cmd.vtheta = vtheta
        cmd.reset = reset
        cmds.append(cmd)
        
        worldData = self.sendCmd(cmds)[0]

        state = self.calculateStateOneWorld(worldData)
        reward,done = self.calculateReward(state)

        return state,reward,done
        # return worldData


    def calculateStateOneWorld(self,worldData):
        # 计算状态：和目标点的距离，和目标点的角度，激光雷达数据，速度，角速度
        distance = []
        angle = []
        laser_data = []
        line_speed = []
        angle_speed = []
        stalled = []
        if len(worldData.robotId) != self.getRobotNumber():
            raise ValueError("机器人数量不一致")

        #计算状态

        for i in range(len(worldData.robotId)):
            
            x1,y1 = self.goal[i][0] - worldData.x[i] , self.goal[i][1] - worldData.y[i]
            x2,y2 = math.cos(worldData.theta[i] + math.pi) , math.sin(worldData.theta[i] + math.pi) #因为stage中朝向-x轴是0度，所以这里要加上pi
            
            distance.append((x1**2 + y1**2)**0.5)#距离
            
            #计算角度
            # 计算点积
            dot_product = x1 * x2 + y1 * y2
            # 计算叉积（用行列式表示）
            det = x1 * y2 - y1 * x2
            # 使用 atan2 计算夹角
            angle_get = math.atan2(det, dot_product)  # 返回值范围在 (-π, π)


            angle.append(angle_get)
            laser_data.append(worldData.laserData[i])
            line_speed.append(worldData.vx[i])
            angle_speed.append(worldData.vtheta[i])
            stalled.append(worldData.isStalled[i])

        state = {"distance":distance,"angle":angle,"laser_data":laser_data,"line_speed":line_speed,"angle_speed":angle_speed,"stalled":stalled}
        return state


    def calculateReward(self,state):
        #计算奖励
        distance = state["distance"]
        angle_speed = state["angle_speed"]
        stalled = state["stalled"]
        reward = []
        done = []

        if self.last_distance is None:  # 第一次调用
                self.last_distance = distance

        if len(distance) != self.getRobotNumber():
            raise ValueError("机器人数量不一致")


        for i in range(self.getRobotNumber()):
            rew = 0
            d = False
            reward_global = 0

            if distance[i] < 0.5:
                reward_global = REWARD_ARRIVE
                d = True
            else :
                reward_global = REWARD_WG * (self.last_distance[i] - distance[i])

            #撞车惩罚
            reward_collides = 0 
            if stalled[i] == True:
                reward_collides = REWARD_COLLISION
                d = True
            
            #平滑移动奖励函数
            reward_smooth = 0
            if abs(angle_speed[i]) > 0.7:
                reward_smooth = REWARD_WW * abs(angle_speed[i])
            
            rew = reward_global + reward_collides + reward_smooth

            reward.append(rew)
            done.append(d)
        

        self.last_distance = distance
        return reward,done