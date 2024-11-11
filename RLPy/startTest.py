import ppo
import os
p = ppo.PPO(gui = True)
#首先查看模型文件是否存在
if os.path.exists('./ppo_actor.pth') and os.path.exists('./ppo_critic.pth'):
    p.start_test()
else:
    print('模型不存在，请先导入模型！')




    

