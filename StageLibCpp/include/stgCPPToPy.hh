#include <iostream>
#include <Stage-4.3/stage.hh>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
namespace StgCPPToPy {
    extern std::mutex mtx;
    extern std::condition_variable cv;
    extern bool ready;

    const std::string stageFileStr = "stage1.world";
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

    extern std::vector<WorldNode*> worlds; //世界列表
    int callback(Stg::World *world, void* user);
    void pycall(void);

}


