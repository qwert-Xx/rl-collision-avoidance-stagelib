#include <stgCPPToPy.hh>

#include <thread>






int main(int argc,char* argv[]) {
    std::thread t(StgCPPToPy::pycall);
    StgCPPToPy::Init(1);
    t.join();
    return 0;
}


