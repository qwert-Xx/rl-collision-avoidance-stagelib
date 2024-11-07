#include <stgCPPToPy.hh>

#include <thread>





std::vector<StgCPPToPy::RobotCmd> robotCmds;
void callpy(std::vector<StgCPPToPy::RobotCmd> robotCmds){
    while(1)
        StgCPPToPy::pycall(robotCmds);
}

int main(int argc,char* argv[]) {
    //创建一个机器人指令列表
    
    StgCPPToPy::RobotCmd cmd;
    cmd.id = 0;
    for(uint8_t i = 0 ; i < 24 ; i++){
        cmd.robotId.push_back(i);
        cmd.vx.push_back(0.1);
        cmd.vy.push_back(0.1);
        cmd.vtheta.push_back(0.1);
        cmd.reset.push_back(false);
    }
    robotCmds.push_back(cmd);

    StgCPPToPy::WorldData worldData;

    std::thread t(callpy,robotCmds);
    StgCPPToPy::Init(1);
    t.join();
    return 0;
}


