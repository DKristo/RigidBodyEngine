#ifndef RigidBody_h
#define RigidBody_h

#include "Vector2f.h"

class CircleBody;
class PolygonBody;
struct Collision;

class RigidBody {
protected:
    Vector2f _position, _force, _velocity;
    float _radius;
    
    float _orientation, _torque, _angularVelocity;
    float _inverseMass, _inverseMomentOfInertia;
    
    float _restitution;
    float _staticFriction;
    float _dynamicFriction;

public:
    RigidBody(const Vector2f& position, float orientation, float mass, float momentOfInertia, float restitution, float staticFriction, float dynamicFriction);
    virtual ~RigidBody();
    
    void applyForce(const Vector2f& force);
    void applyTorque(float torque);
    void applyImpulse(const Vector2f& impulse, const Vector2f& contactVector);
    
    virtual void integrateForce(float timeStep, const Vector2f& gravity);
    virtual void integrateVelocity(float timeStep);
    void zeroForces();
    
    virtual bool checkCollision(RigidBody& body, Collision& collision) = 0;
    virtual bool checkCollision(CircleBody& circle, Collision& collision) = 0;
    virtual bool checkCollision(PolygonBody& polygon, Collision& collision) = 0;
    
    void resolveCollision(Collision& collision, float timeStep, const Vector2f& gravity);
    void performCollisionAdjustment(Collision& collision);
    
    float radius() const;
    const Vector2f& position() const;
    float orientation() const;
};

#endif
