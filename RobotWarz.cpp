#include "Arena.h"

#include <cstring>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    std::string config_path;

    for (int i = 1; i < argc; ++i) {
        if (config_path.empty()) {
            config_path = argv[i];
        } else {
            std::cerr << "Usage: " << argv[0] << " <config_file>\n";
            return 1;
        }
    }

    if (config_path.empty()) {
        std::cerr << "Usage: " << argv[0] << " <config_file>\n";
        return 1;
    }

    try {
        Arena arena(std::move(config_path));
        arena.run();
    } catch (const std::exception& ex) {
        std::cerr << "RobotWarz error: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
