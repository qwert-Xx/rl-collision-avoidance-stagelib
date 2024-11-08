
import torch
import torch.nn as nn
from torch.nn import init
from torch.nn import functional as F
from torch.optim import Adam
import numpy as np

#定义网络结构
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
class ActorNet(nn.Module):
    def __init__(self):
        super(ActorNet,self).__init__()
        self.act_fea_cv1 = nn.Conv1d(in_channels=3, out_channels=32, kernel_size=5, stride=2, padding=1)
        self.act_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.act_fc1 = nn.Linear(128*32, 256)
        self.act_fc2 =  nn.Linear(256+2+2, 128)
        self.actor1 = nn.Linear(128, 1)
        self.actor2 = nn.Linear(128, 1)

        self.logstd = nn.Parameter(torch.tensor([0.0,0.0])) #线速度和角速度的标准差的对数

    def forward(self, laser_data, line_speed, angle_speed, distance, angle):
        if isinstance(laser_data, np.ndarray):
            laser_data = torch.tensor(laser_data, dtype=torch.float)
        if isinstance(line_speed, np.ndarray):
            line_speed = torch.tensor(line_speed, dtype=torch.float)
        if isinstance(angle_speed, np.ndarray):
            angle_speed = torch.tensor(angle_speed, dtype=torch.float)
        if isinstance(distance, np.ndarray):
            distance = torch.tensor(distance, dtype=torch.float)
        if isinstance(angle, np.ndarray):
            angle = torch.tensor(angle, dtype=torch.float)
            
        x = F.relu(self.act_fea_cv1(laser_data))
        x = F.relu(self.act_fea_cv2(x))
        x = x.view(x.shape[0],-1)
        x = F.relu(self.act_fc1(x))
        x = torch.cat([x, line_speed, angle_speed, distance, angle] , dim = -1)
        x = F.relu(self.act_fc2(x))
        line_speed_out = torch.sigmoid(self.actor1(x)) # 输出在 [0, 1] 范围内
        angle_speed_out = torch.tanh(self.actor2(x))  # 输出在 [-1, 1] 范围内
        out = torch.cat((line_speed_out, angle_speed_out), dim=1)
        
        #输出线速度限制到0到1，角速度限制到-1到1
        return out,self.logstd #输出为线速度和角速度的均值 和 标准差的对数

class CriticNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.val_fea_cv1 = nn.Conv1d(in_channels=3, out_channels=32, kernel_size=5, stride=2, padding=1)
        self.val_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.val_fc1 = nn.Linear(128*32, 256)
        self.val_fc2 =  nn.Linear(256+2+2, 128)
        self.value = nn.Linear(128, 1)

    def forward(self, laser_data, line_speed, angle_speed, distance, angle):
        if isinstance(laser_data, np.ndarray):
            laser_data = torch.tensor(laser_data, dtype=torch.float)
        if isinstance(line_speed, np.ndarray):
            line_speed = torch.tensor(line_speed, dtype=torch.float)
        if isinstance(angle_speed, np.ndarray):
            angle_speed = torch.tensor(angle_speed, dtype=torch.float)
        if isinstance(distance, np.ndarray):
            distance = torch.tensor(distance, dtype=torch.float)
        if isinstance(angle, np.ndarray):
            angle = torch.tensor(angle, dtype=torch.float)
        x = F.relu(self.val_fea_cv1(laser_data))
        x = F.relu(self.val_fea_cv2(x))
        x = x.view(x.shape[0],-1)
        x = F.relu(self.val_fc1(x))
        x = torch.cat([x, line_speed, angle_speed, distance, angle],dim = -1)
        x = F.relu(self.val_fc2(x))
        return self.value(x) #输出为状态价值
    

