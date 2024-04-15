#include "states.hpp"
#include <string>

void stateSwitch(State curState, std::string command) {
    switch (curState) {
    case State::init:
        init();
        break;
    case State::idle:
        idle();
        break;
    case State::precharge:
        precharge();
        break;
    case State::ready:
        ready(command);
        break;
    case State::start:
        start();
        break;
    case State::running:
        running();
        break;
    case State::stop:
        stop();
        break;
    }
}