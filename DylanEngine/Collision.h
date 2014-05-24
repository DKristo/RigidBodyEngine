#ifndef Collision_h
#define Collision_h

#include <vector>
#include "Vector2f.h"
#include "RigidBody.h"

struct Collision {
    RigidBody* bodyA;
    RigidBody* bodyB;
    
    Vector2f normal;
    float depth;
    
    std::vector<Vector2f> contactPoints;
};

#endif
