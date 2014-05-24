#ifndef Timer_h
#define Timer_h

#include <SFML/System.hpp>

class Timer {
private:
    sf::Clock _clock;
    float _millisPerUpdate, _timeAccumulator;
    
public:
    Timer(float updatesPerSecond);
    ~Timer() {}
    
    void start();
    void update();
    
    bool checkTimeStep();
    float timeStep() const;
};

#endif
