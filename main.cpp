#include "tcp_server.h"
#include "physics.h"
#include "renderer.h"

#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[])
{
    int port = 8080;

    if(argc >= 2)
        port = std::atoi(argv[1]);

    std::cout << "\n";
    std::cout << "=========================================\n";
    std::cout << " Omni Flight Vehicle Simulator\n";
    std::cout << "=========================================\n";
    std::cout << "[SIM] Physics rate : 120 Hz\n";
    std::cout << "[SIM] Socket rate  : 50 Hz\n";
    std::cout << "[SIM] Coordinate   : EUS\n";
    std::cout << "[SIM] Motors       : 4\n";
    std::cout << "[SIM] Servo Joint1 : 4\n";
    std::cout << "[SIM] Servo Joint2 : 4\n";
    std::cout << "[SIM] Total Actuators : 12\n";
    std::cout << "[SIM] Press R to reset\n";
    std::cout << "=========================================\n";

    SimState state;
    state.reset();

    TcpServer server(port, state);
    server.start();

    Renderer renderer(state);
    renderer.run();

    server.stop();

    return 0;
}