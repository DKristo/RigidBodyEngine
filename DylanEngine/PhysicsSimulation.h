#ifndef PhysicsSimulation_h
#define PhysicsSimulation_h

#include <vector>
#include "CircleBody.h"

class RigidBody;

class PhysicsSimulation {
private:
    Vector2f _gravity;
    unsigned int _numIterations;
    std::vector<RigidBody*> _bodies;

public:
    PhysicsSimulation(const Vector2f& gravity, unsigned int numIterations);
    ~PhysicsSimulation();
    
    void addBody(RigidBody* body);
    void removeBody(RigidBody* body);
    void update(float timeStep);
    
    const std::vector<RigidBody*>& bodies() const;
};

#endif
