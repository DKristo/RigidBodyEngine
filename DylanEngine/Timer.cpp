#include "Timer.h"

Timer::Timer(float millisecondsPerUpdate) :
_millisPerUpdate(millisecondsPerUpdate),
_timeAccumulator(0.0f) {}

void Timer::start() {
    _timeAccumulator = 0.0f;
    _clock.restart();
}

void Timer::update() {
    float deltaTime = _clock.restart().asSeconds() * 1000.0f;
    
    _timeAccumulator += deltaTime;
    
    //Set a limit on how much time can accumulate, to limit the number of simultaneous updates
    //that can be performed on really slow systems that can't keep up
    if (_timeAccumulator > 500.0f)
        _timeAccumulator = 500.0f;
}

bool Timer::checkTimeStep() {
    if (_timeAccumulator >= _millisPerUpdate) {
        _timeAccumulator -= _millisPerUpdate;
        return true;
    }
    
    return false;
}

float Timer::timeStep() const {
    return _millisPerUpdate;
}