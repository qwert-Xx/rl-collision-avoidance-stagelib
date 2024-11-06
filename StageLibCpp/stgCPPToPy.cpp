#include <stgCPPToPy.hh>




namespace StgCPPToPy{
    std::vector<WorldNode*> worlds; //世界列表

    int Init(uint8_t num_world){
        std::cout << "Start,World!" << std::endl;
        std::cout << "World Number :" << num_world << std::endl;
        int argc = 0;
        char** argv = NULL;

        
        Stg::Init(&argc, &argv);

        //初始化world,并添加到worlds列表中

        if(num_world == 1){
            Stg::World* world = new Stg::WorldGui(600,400,"Test Stage");
            world->Load(stageFileStr);
            world->ShowClock(false);
            if (!world->paused)
                world->Start();
            worlds.push_back(new WorldNode(world,0) );
        }else{
            for (uint8_t i = 0; i < num_world; i++){
                Stg::World* world = new Stg::World();
                world->Load(stageFileStr);
                world->ShowClock(false);
                if (!world->paused)
                    world->Start();
                worlds.push_back(new WorldNode(world,i));
            }
        }



        //添加各种功能 #TODO
        
        for (int i = 0; i < worlds.size(); i++){
            WorldNode* world = worlds[i];
            //添加机器人
            
            //添加回调函数
            world->AddUpdateCallback(callback);
            std::cout << "World " << static_cast<int>(world->GetId()) << " Init Success!" << std::endl;

        }



        //启动
        Stg::World::Run();
        std::cout << "End, World!" << std::endl;
        return 0;


    }

    



    Stg::Pose Robot::GetPositionData(void){
        Stg::ModelPosition* position = this->position;
        Stg::Pose pose = position->GetGlobalPose();
        return pose;
    }

    Stg::Velocity Robot::GetSpeedData(){
        Stg::ModelPosition* position = this->position;
        Stg::Velocity velocity = position->GetGlobalVelocity();
        return velocity;
    }

    bool Robot::SetSpeedData(Stg::Velocity velocity){
        Stg::ModelPosition* position = this->position;
        position->SetSpeed(velocity);
        return true;
    }

    std::deque<std::vector<Stg::meters_t>> Robot::GetLaserData(void){
        Stg::ModelRanger* ranger = this->ranger;
        Stg::ModelRanger::Sensor sensor = ranger->GetSensors()[0];
        Stg::radians_t fov  = sensor.fov;
        std::vector<Stg::meters_t> ranges = sensor.ranges;
        while(this->rangesData.size() < 3){ //当历史数据不够时直接使用当前数据填充
            this->rangesData.push_back(ranges);
        }

        this->rangesData.pop_front();
        this->rangesData.push_back(ranges);
        if(this->rangesData.size() != 3){
            std::cout << "Error: rangesData size != 3" << std::endl;
            //抛出异常
            throw "Error: rangesData size != 3";
        }
        return this->rangesData;
    }

    void Robot::ResetPosition(void){
        Stg::ModelPosition* position = this->position;
        position->SetPose(this->initialPose);
        return;
    }

    std::string Robot::GetName(void){
        return this->position->TokenStr();
    }

    int callback(Stg::World *world, void* user){
        WorldNode* node = (WorldNode*)user;
        // std::cout << "Callback from world:" << static_cast<int>(node->GetId()) << std::endl;
        // std::cout << "Time:" << world->SimTimeNow() << std::endl;
        //遍历每一个机器人
        for (Robot* robot : node->GetRobots()){
            
        }
        return 0;

    }

}

