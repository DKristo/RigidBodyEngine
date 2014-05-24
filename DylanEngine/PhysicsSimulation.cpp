#include "PhysicsSimulation.h"
#include "Collision.h"

PhysicsSimulation::PhysicsSimulation(const Vector2f& gravity, unsigned int numIterations) :
_gravity(gravity),
_numIterations(numIterations) {}

PhysicsSimulation::~PhysicsSimulation() {}

void PhysicsSimulation::addBody(RigidBody* body) {
    _bodies.push_back(body);
}

void PhysicsSimulation::removeBody(RigidBody* body) {
    for (std::vector<RigidBody*>::iterator i = _bodies.begin(); i != _bodies.end(); ++i) {
        if ((*i) == body) {
            _bodies.erase(i);
            return;
        }
    }
}

void PhysicsSimulation::update(float timeStep) {
    std::vector<Collision> collisions;
    Collision collisionData;
    
    //Check for collisions, store collision data for detected collisions
    for (size_t i = 0; i < _bodies.size() - 1; ++i)
        for (size_t j = i + 1; j < _bodies.size(); ++j)
            if (_bodies[i]->checkCollision(*_bodies[j], collisionData))
                collisions.push_back(collisionData);
    
    //Integrate force into velocity
    for (std::vector<RigidBody*>::iterator i = _bodies.begin(); i != _bodies.end(); ++i)
        (*i)->integrateForce(timeStep, _gravity);
    
    //Iterate and apply impulses to resolve collisions
    for (unsigned int i = 0; i < _numIterations; ++i) {
        for (std::vector<Collision>::iterator j = collisions.begin(); j != collisions.end(); ++j)
            j->bodyA->resolveCollision((*j), timeStep, _gravity);
    }
    
    //Integrate velocity into position and reset forces to zero
    for (std::vector<RigidBody*>::iterator i = _bodies.begin(); i != _bodies.end(); ++i) {
        (*i)->integrateVelocity(timeStep);
        (*i)->zeroForces();
    }
    
    for (std::vector<Collision>::iterator i = collisions.begin(); i != collisions.end(); ++i)
        i->bodyA->performCollisionAdjustment((*i));
}

const std::vector<RigidBody*>& PhysicsSimulation::bodies() const {
    return _bodies;
}