#include <string>

enum class State { init, idle, precharge, ready, start, running, stop };

State init();

State idle();

State precharge();

State ready(std::string);

State start();

State running();

State stop();