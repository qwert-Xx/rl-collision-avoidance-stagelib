#include <iostream>
#include <Stage-4.3/stage.hh>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
namespace StgCPPToPy {
    extern std::mutex mtx;
    extern std::condition_variable cv;
    extern bool ready;
    extern std::thread* mainThread;

    const std::string stageFileStr = "stage1.world";
    void Start(uint8_t num_world);//在这个函数中开启新线程调用Init
    int Init(uint8_t num_world);

    class Robot{
        public:
            Robot(Stg::World* world , Stg::ModelPosition* position , uint8_t id){
                this->world = world;
                this->id = id;
                this->position = position;
                if(position->GetSubscriptionCount() == 0){
                    position->Subscribe();
                }
                this->initialPose = position->GetGlobalPose();
                if(position->GetChild("ranger:0") != NULL){
                    Stg::ModelRanger* ranger = dynamic_cast<Stg::ModelRanger*>(position->GetChild("ranger:0"));
                    this->ranger = ranger;
                    if(ranger->GetSubscriptionCount() == 0){
                        ranger->Subscribe();
                    }
                }

            }
            Stg::Pose GetPositionData(void);
            Stg::Velocity GetSpeedData(void);
            bool SetSpeedData(Stg::Velocity);
            std::deque<std::vector<Stg::meters_t>> GetLaserData(void);
            void ResetPosition(void);
            std::string GetName(void);
            Stg::World* GetWorld(void){
                return this->world;
            }
            uint8_t GetId(void){
                return this->id;
            }
        private:
            Stg::World* world;
            Stg::ModelPosition* position;
            Stg::ModelRanger* ranger;
            Stg::Pose initialPose;
            std::deque<std::vector<Stg::meters_t>> rangesData; //历史的激光雷达数据
            uint8_t id;
            Stg::Pose lastPose;
    };

    class WorldNode{
        public:
            
            WorldNode(Stg::World* world,uint8_t id){
                this->world = world;
                this->id = id;
                //添加机器人
                std::set<Stg::Model*> models = world->GetAllModels();
                uint8_t id_robots = 0;
                for (Stg::Model* model : models) {
                    // Process each model
                    //订阅每个模型
                    model->Subscribe();
                    if(model->GetModelType() == "position"){
                        Stg::ModelPosition* position = dynamic_cast<Stg::ModelPosition*>(model);
                        Robot* robot = new Robot(world,position,id_robots);
                        this->robots.push_back(robot);
                        id_robots++;
                    }
                }
            }
            Stg::World* GetWorld(void){
                return this->world;
            }
            uint8_t GetId(void){
                return this->id;
            }
            bool AddUpdateCallback(Stg::world_callback_t callback){
                this->world->AddUpdateCallback(callback, this);
                return true;
            }
            std::vector<Robot*> GetRobots(void){
                return this->robots;
            }
        private:
            Stg::World* world;
            std::vector<Robot*> robots;
            uint8_t id;
    };

    //pycall返回值类型，包含每个世界每个机器人的位置、速度、激光雷达数据
    struct WorldData{ //由机器人数据组成的向量,每个元素代表一个机器人，
        uint8_t id;
        std::vector<std::string> name;//机器人名字
        std::vector<uint8_t> robotId;//机器人id 
        std::vector<double> x;//位置x
        std::vector<double> y;//位置y
        std::vector<double> theta;//角度
        std::vector<double> vx;//速度x
        std::vector<double> vy;//速度y
        std::vector<double> vtheta;//角速度
        std::vector<std::deque<std::vector<Stg::meters_t>>> laserData;//激光雷达数据
    };
    
    struct RobotCmd{//每个机器人的速度指令
        uint8_t id;//世界id
        std::vector<uint8_t> robotId;//机器人id
        std::vector<double> vx;//速度x
        std::vector<double> vy;//速度y
        std::vector<double> vtheta;//角速度
        std::vector<bool> reset;//是否重置位置
    };

    extern std::vector<WorldNode*> worlds; //世界列表
    int callback(Stg::World *world, void* user);
    std::vector<WorldData> pycall(std::vector<RobotCmd>);

    PYBIND11_MODULE(stgCPPToPy,m){
        py::class_<RobotCmd>(m,"RobotCmd")
            .def(py::init<>())
            .def_readwrite("id",&RobotCmd::id , "世界id")
            .def_readwrite("robotId",&RobotCmd::robotId , "机器人id")
            .def_readwrite("vx",&RobotCmd::vx , "速度x")
            .def_readwrite("vy",&RobotCmd::vy , "速度y")
            .def_readwrite("vtheta",&RobotCmd::vtheta , "角速度")
            .def_readwrite("reset",&RobotCmd::reset , "是否重置位置");
        py::class_<WorldData>(m,"WorldData")
            .def(py::init<>())
            .def_readwrite("id",&WorldData::id , "世界id")
            .def_readwrite("name",&WorldData::name , "机器人名字")
            .def_readwrite("robotId",&WorldData::robotId , "机器人id")
            .def_readwrite("x",&WorldData::x , "位置x")
            .def_readwrite("y",&WorldData::y , "位置y")
            .def_readwrite("theta",&WorldData::theta , "角度")
            .def_readwrite("vx",&WorldData::vx , "速度x")
            .def_readwrite("vy",&WorldData::vy , "速度y")
            .def_readwrite("vtheta",&WorldData::vtheta , "角速度")
            .def_readwrite("laserData",&WorldData::laserData , "激光雷达数据");
        m.def("Start",&Start);
        m.def("pycall",&pycall);
    }
}


