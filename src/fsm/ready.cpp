#include "states.hpp"
#include <string>

State ready(std::string command) {
    if (command == "start") {
        return State::start;
    } else {
        return State::idle;
    }
}
