#include <iostream>
#include <Stage-4.3/stage.hh>
Stg::World*  world;
std::string stageFileStr = "stage1.world";

int main(int argc,char* argv[]) {
    std::cout << "Start, World!" << std::endl;
    Stg::Init(&argc, &argv);
    world = new Stg::WorldGui(600,400,"Test Stage");
    world->Load(stageFileStr);
    world->ShowClock(false);
    if (!world->paused)
        world->Start();

    Stg::World::Run();
    std::cout << "End, World!" << std::endl;

    return 0;
}