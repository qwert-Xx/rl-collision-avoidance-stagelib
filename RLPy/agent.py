import stgCPPToPy
import math
import random

REWARD_ARRIVE = 15  #到达目标点时的奖励
REWARD_WG = 2.5  #向目标点靠近的奖励系数

REWARD_COLLISION = -15  #碰撞的惩罚

REWARD_WW = -0.1  #角速度过大的惩罚系数
class WorldDataForPython():
    def __init__(self) -> None:
        self.id = 0
        self.name = ""
        self.robotId = []
        self.x = []
        self.y = []
        self.theta = []
        self.vx = []
        self.vy = []
        self.vtheta = []
        self.laserData = []
        self.isStalled = []
        pass
class Agent():
    def __init__(self,goal,agent_num = 24,num_world = 1,gui = True) -> None:
        if num_world != 1:
            raise ValueError("暂未实现多于1个世界")
        stgCPPToPy.Start(num_world,gui)
        self.agent_num = agent_num
        self.goal = goal + [[0,0] for i in range(self.getRealRobotNumber() - len(goal))]#目标点,这是一个二维列表[robots][x,y]

        
        self.last_distance = None

    def sendCmd(self,cmd):#发送命令，即发送命令，返回执行的结果
        worldData = stgCPPToPy.pycall(cmd)[0]
        #将返回的值转换成python原生类型
        worldDataPy = WorldDataForPython()
        worldDataPy.id = worldData.id
        worldDataPy.name = worldData.name.copy()
        worldDataPy.robotId = worldData.robotId.copy()
        worldDataPy.x = worldData.x.copy()
        worldDataPy.y = worldData.y.copy()
        worldDataPy.theta = worldData.theta.copy()
        worldDataPy.vx = worldData.vx.copy()
        worldDataPy.vy = worldData.vy.copy()
        worldDataPy.vtheta = worldData.vtheta.copy()
        worldDataPy.laserData = worldData.laserData.copy()
        worldDataPy.isStalled = worldData.isStalled.copy()

        return worldDataPy
    def getRobotNumber(self):
        return self.agent_num
    def getRealRobotNumber(self):
        return 24
    def reset(self):
        #设置目标点
        goalPositions = []
        goalPosition = stgCPPToPy.RobotPosition()
        goalPosition.id = 0
        goalPosition.robotId = list(range(self.getRealRobotNumber()))
        goalPosition.x = [x for x,y in self.goal]
        goalPosition.y = [y for x,y in self.goal]
        goalPosition.theta = [0 for _ in range(self.getRealRobotNumber())]
        goalPositions.append(goalPosition)
        stgCPPToPy.SetRobotGoalPosition(goalPositions)

        #设置机器人初始动作
        Cmds = []
        cmd = stgCPPToPy.RobotCmd()
        worldId = 0
        robotId = list(range(self.getRealRobotNumber()))
        vx = [0.0 for i in range(self.getRealRobotNumber())]
        vy = [0.0 for i in range(self.getRealRobotNumber())]
        vtheta = [0.0 for i in range(self.getRealRobotNumber())]
        reset = [True for i in range(self.getRealRobotNumber())]
        cmd.id = worldId
        cmd.robotId = robotId
        cmd.vx = vx
        cmd.vy = vy
        cmd.vtheta = vtheta
        cmd.reset = reset
        Cmds.append(cmd)
        self.last_distance = None
        worldData =  self.sendCmd(Cmds)
        state = self.calculateStateOneWorld(worldData)
        state["distance"] = state["distance"][:self.getRobotNumber()]
        state["angle"] = state["angle"][:self.getRobotNumber()]
        state["laser_data"] = state["laser_data"][:self.getRobotNumber()]
        state["line_speed"] = state["line_speed"][:self.getRobotNumber()]
        state["angle_speed"] = state["angle_speed"][:self.getRobotNumber()]
        state["stalled"] = state["stalled"][:self.getRobotNumber()]

        return state
        


    def step(self ,action):

        vx = action["line_speed"] + [0 for i in range(self.getRealRobotNumber() - len(action["line_speed"])) ] 
        vtheta = action["angle_speed"] + [0 for i in range(self.getRealRobotNumber() - len(action["line_speed"])) ] 
        cmds = []
        cmd = stgCPPToPy.RobotCmd()
        worldId = 0
        robotId = list(range(self.getRealRobotNumber()))
        vy = [0.0 for i in range(self.getRealRobotNumber())]
        reset = [False for i in range(self.getRealRobotNumber())]
        cmd.id = worldId
        cmd.robotId = robotId
        cmd.vx = vx
        cmd.vy = vy
        cmd.vtheta = vtheta
        cmd.reset = reset
        cmds.append(cmd)
        
        worldData = self.sendCmd(cmds)

        state = self.calculateStateOneWorld(worldData)
        reward,done = self.calculateReward(state)

        state["distance"] = state["distance"][:self.getRobotNumber()]
        state["angle"] = state["angle"][:self.getRobotNumber()]
        state["laser_data"] = state["laser_data"][:self.getRobotNumber()]
        state["line_speed"] = state["line_speed"][:self.getRobotNumber()]
        state["angle_speed"] = state["angle_speed"][:self.getRobotNumber()]
        state["stalled"] = state["stalled"][:self.getRobotNumber()]

        reward = reward[:self.getRobotNumber()]
        done = done[:self.getRobotNumber()]

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
        if len(worldData.robotId) != self.getRealRobotNumber():
            raise ValueError("机器人数量不一致")

        #计算状态

        for i in range(len(worldData.robotId)):
            
            x1,y1 = self.goal[i][0] - worldData.x[i] , self.goal[i][1] - worldData.y[i]
            x2,y2 = math.cos(worldData.theta[i]) , math.sin(worldData.theta[i]) 
            
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

        if len(distance) != self.getRealRobotNumber():
            raise ValueError("机器人数量不一致")


        for i in range(self.getRealRobotNumber()):
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
    def randomSetInitPosition(self,radius = 10,distance = 2):
        self.init_positions = []
        for _ in range(self.getRealRobotNumber()):
            while True:
                x = random.uniform(-radius, radius)
                y = random.uniform(-radius, radius)
                if all(math.hypot(x - ix, y - iy) > distance for ix, iy in self.init_positions):
                    self.init_positions.append([x, y])
                    break
        robotPositionDatas = []
        robotPositionData = stgCPPToPy.RobotPosition()
        robotPositionData.id = 0
        robotPositionData.robotId = list(range(self.getRealRobotNumber()))
        robotPositionData.x = [x for x,y in self.init_positions]
        robotPositionData.y = [y for x,y in self.init_positions]
        robotPositionData.theta = [random.uniform(-math.pi, math.pi) for _ in range(self.getRealRobotNumber())]
        robotPositionDatas.append(robotPositionData)
        stgCPPToPy.SetRobotInitPosition(robotPositionDatas)
        
    def randomSetGoal(self,radius = 10 ,distance = 2):
        #生成随机目标点，每个目标点之间的距离大于distance,范围为-radius到radius
        self.goal = []
        for _ in range(self.getRealRobotNumber()):
            while True:
                x = random.uniform(-radius, radius)
                y = random.uniform(-radius, radius)
                if all(math.hypot(x - gx, y - gy) > distance for gx, gy in self.goal):
                    self.goal.append([x, y])
                    break
            