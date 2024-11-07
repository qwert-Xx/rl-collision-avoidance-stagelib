#include <stgCPPToPy.hh>




namespace StgCPPToPy{
    std::mutex mtx;
    std::condition_variable cv;
    bool ready = true;
    std::vector<WorldNode*> worlds; //世界列表
    std::thread* mainThread; //主线程

    void Start(uint8_t num_world){
        mainThread =  new std::thread(StgCPPToPy::Init,num_world);
    }

    int Init(uint8_t num_world){
        std::cout << "Start,World!" << std::endl;
        std::cout << "World Number :" << static_cast<int>(num_world) << std::endl;
        int* argc = new int(0);        
        Stg::Init(argc, NULL);
        std::cout << "Init Success!" << std::endl;
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
        
        for (std::size_t i = 0; i < worlds.size(); i++){
            WorldNode* world = worlds[i];
            //添加机器人
            
            //添加回调函数
            world->AddUpdateCallback(callback);
            std::cout << "World " << static_cast<int>(world->GetId()) << " Init Success!" << std::endl;

        }



        //启动一个线程来启动
        // mainThread = new std::thread(Stg::World::Run);
        Stg::World::Run();
        
        std::cout << "End, Init!" << std::endl;
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
        // Stg::radians_t fov  = sensor.fov;
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

    int callback(Stg::World *world, void* user){//世界更新回调函数
        WorldNode* node = (WorldNode*)user;
        // std::cout << "Callback from world:" << static_cast<int>(node->GetId()) << std::endl;
        // std::cout << "Time:" << world->SimTimeNow() << std::endl;


        //这里进行各种处理

        



        
        if(node->GetId() == worlds.size() - 1){
            //处理完了最后一个世界后唤醒pycall
            std::unique_lock<std::mutex> lck(mtx);
            ready = false;
            cv.notify_all();
            lck.unlock();

            //唤醒pycall之后等待pycall执行完毕
            lck.lock();
            cv.wait(lck, []{return ready;});
            lck.unlock();
        }
        return 0;
    }

    std::vector<WorldData> pycall(std::vector<RobotCmd> robotCmds){
        //python调用这个函数，用于更新和获取数据
        //每次遍历完所有的world之后，需要调用pycall之后才能继续下一次循环
        std::unique_lock<std::mutex> lck(mtx);
        cv.wait(lck, []{return !ready;});
        ready = true;

        //在这里进行处理
        std::vector<WorldData> worldsData;
        //遍历每个世界
        for (std::size_t i = 0; i < worlds.size(); i++){
            WorldNode* world = worlds[i];
            WorldData singalWorldData;
            RobotCmd cmd = robotCmds[i];
            singalWorldData.id = world->GetId();
            if(cmd.id != world->GetId()){
                std::cout << "Error: World id not match!" << std::endl;
                //抛出异常
                throw "Error: World id not match!";
            }
            //遍历每个机器人
            
            std::vector<Robot*> robots = world->GetRobots();
            for (std::size_t j = 0; j < robots.size(); j++){
                Robot* robot = robots[j];
                //处理每个机器人
                Stg::Pose pose = robot->GetPositionData();
                Stg::Velocity velocity = robot->GetSpeedData();
                std::deque<std::vector<Stg::meters_t>> ranges = robot->GetLaserData();
                
                singalWorldData.x.push_back(pose.x);
                singalWorldData.y.push_back(pose.y);
                singalWorldData.theta.push_back(pose.a);
                singalWorldData.vx.push_back(velocity.x);
                singalWorldData.vy.push_back(velocity.y);
                singalWorldData.vtheta.push_back(velocity.a);
                singalWorldData.laserData.push_back(ranges);
                singalWorldData.robotId.push_back(robot->GetId());
                singalWorldData.name.push_back(robot->GetName());
                //处理cmd
                if(cmd.robotId[j] != robot->GetId()){
                    std::cout << "Error: Robot id not match!" << std::endl;
                    //抛出异常
                    throw "Error: Robot id not match!";
                }
                Stg::Velocity newVelocity;
                newVelocity.x = cmd.vx[j];
                newVelocity.y = cmd.vy[j];
                newVelocity.a = cmd.vtheta[j];
                robot->SetSpeedData(newVelocity);
                if(cmd.reset[j]){
                    robot->ResetPosition();
                }
            }
            worldsData.push_back(singalWorldData);
        }    



        cv.notify_all();//处理完了之后再唤醒
        lck.unlock();
        // std::this_thread::sleep_for(std::chrono::seconds(3)); // 阻塞3秒
        return worldsData;
    }
}

