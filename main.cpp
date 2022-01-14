#include <math.h>
#include <iostream>
#include <fmt/format.h>

#include <App/Axis.h>

#include <App/Controller.h>
#include <A3200.h>

using namespace fmt;

void printerror();

void printerror() {
    char data[1024];
    A3200GetLastErrorString(data, 1024);
    std::cout << std::printf("Error : %s\n", data);
}

int main(int argc, char *argv[])
{

    std::string s = fmt::format("The answer is {} \n", 42);

    auto contr = std::make_shared<aerotech::A3200Controller>();
    auto X = std::make_shared<aerotech::Axis>(contr,1);
    auto Y = std::make_shared<aerotech::Axis>(contr,2);

    uint32_t mask = contr->getAxisMask(std::vector<aerotech::Axis::Ptr>{X,Y});

    // Handle to give access to A3200
    A3200Handle handle = nullptr;

    // Number of lines left in the queue to execute

    printf("Connecting to A3200. Initializing if necessary.\n");
    if(!A3200Connect(&handle)) {
        printerror();
    }


    if(argc < 2) {
        std::cerr << "Number of argument must be one ";
    }


    std::string path = argv[0];
    std::string mode = argv[1];
    std::string inputFilename = argv[2];
    std::string outputFilename = argv[3];
    
}
