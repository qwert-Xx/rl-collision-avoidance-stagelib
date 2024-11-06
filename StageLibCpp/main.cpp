#include <stgCPPToPy.hh>












int main(int argc,char* argv[]) {
    StgCPPToPy::Init(1);

    return 0;
}


std::vector<Stg::World*> StgCPPToPy::worlds;

int StgCPPToPy::Init(uint8_t num_world){
    std::cout << "Start,World!" << std::endl;
    std::cout << "World Number :" << num_world << std::endl;
    int argc = 0;
    char** argv = NULL;

    Stg::World*  world;
    Stg::Init(&argc, &argv);

    //初始化world,并添加到worlds列表中

    if(num_world == 1){
        world = new Stg::WorldGui(600,400,"Test Stage");
        world->Load(StgCPPToPy::stageFileStr);
        world->ShowClock(false);
        if (!world->paused)
            world->Start();
        worlds.push_back(world);
    }else{
        for (int i = 0; i < num_world; i++){
            Stg::World* world = new Stg::World();
            world->Load(StgCPPToPy::stageFileStr);
            world->ShowClock(false);
            if (!world->paused)
                world->Start();
            worlds.push_back(world);
        }
    }



    //添加各种功能 #TODO

    


    //启动
    Stg::World::Run();
    std::cout << "End, World!" << std::endl;
    return 0;


}