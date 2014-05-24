#include "RigidBody.h"
#include <algorithm>
#include "Collision.h"

RigidBody::RigidBody(const Vector2f& position,
                     float orientation,
                     float mass,
                     float momentOfInertia,
                     float restitution,
                     float staticFriction,
                     float dynamicFriction) :
_position(position),
_radius(0.0f),
_orientation(orientation),
_torque(0.0f),
_angularVelocity(0.0f),
_restitution(restitution),
_staticFriction(staticFriction),
_dynamicFriction(dynamicFriction) {
    _inverseMass = (mass < std::numeric_limits<float>::epsilon()) ? 0.0f : (1.0f / mass);
    _inverseMomentOfInertia = (momentOfInertia < std::numeric_limits<float>::epsilon()) ? 0.0f : (1.0f / momentOfInertia);
}

RigidBody::~RigidBody() {}

void RigidBody::applyForce(const Vector2f& force) {
    _force += force;
}

void RigidBody::applyTorque(float torque) {
    _torque = torque;
}

void RigidBody::applyImpulse(const Vector2f& impulse, const Vector2f& contactVector) {
    if (_inverseMass != 0.0f)
        _velocity += impulse * _inverseMass;
    
    if (_inverseMomentOfInertia != 0.0f)
        _angularVelocity += (contactVector.x * impulse.y - contactVector.y * impulse.x) * _inverseMomentOfInertia;
}

void RigidBody::integrateForce(float timeStep, const Vector2f& gravity) {
    if (_inverseMass != 0.0f) {
        _velocity += _force * _inverseMass * timeStep;
        _velocity += gravity * timeStep;
    }
    
    if (_inverseMomentOfInertia != 0.0f)
        _angularVelocity += _torque * _inverseMomentOfInertia * timeStep;
}

void RigidBody::integrateVelocity(float timeStep) {
    if (_inverseMass != 0.0f)
        _position += _velocity * timeStep;
    
    if (_inverseMomentOfInertia != 0.0f)
        _orientation += _angularVelocity * timeStep;    
}

void RigidBody::zeroForces() {
    _force = Vector2f(0.0f, 0.0f);
    _torque = 0.0f;
}

void RigidBody::resolveCollision(Collision& collision, float timeStep, const Vector2f& gravity) {
    RigidBody& b = *(collision.bodyB);
    
    Vector2f velocityA = _velocity;
    Vector2f velocityB = b._velocity;
     
    float angularVelocityA = _angularVelocity;
    float angularVelocityB = b._angularVelocity;
    
    for (std::vector<Vector2f>::const_iterator i = collision.contactPoints.begin(); i != collision.contactPoints.end(); ++i) {
        const Vector2f& contact = *i;
        
        //Vectors from the contact point to center of mass
        Vector2f radiusA = _position - contact;
        Vector2f radiusB = b._position - contact;
        
        //Perpendicular vectors
        Vector2f radiusAPerp(-radiusA.y, radiusA.x);
        Vector2f radiusBPerp(-radiusB.y, radiusB.x);
        
        Vector2f relativeVelocity = velocityB + radiusBPerp * angularVelocityB - velocityA - radiusAPerp * angularVelocityA;
        
        //We use a restitution of 0 to to permit stacking when gravity is the only force acting on a body
        float effectiveRestitution = (relativeVelocity.lengthSquared() < (gravity * timeStep).lengthSquared() + 0.001f) ? 0.0f : std::min(_restitution, b._restitution);

        //The relative speed along the collision normal
        float speed = relativeVelocity.dotProduct(collision.normal);
        
        //Bodies are moving in opposing directions, no need to resolve
        if (speed >= 0.0f)
            return;
        
        float radiusAPerpNormal = radiusAPerp.dotProduct(collision.normal);
        float radiusBPerpNormal = radiusBPerp.dotProduct(collision.normal);
        
        float denominator = (_inverseMass + b._inverseMass + radiusAPerpNormal * radiusAPerpNormal * _inverseMomentOfInertia + radiusBPerpNormal * radiusBPerpNormal * b._inverseMomentOfInertia) * collision.contactPoints.size();
        
        //The magnitude of the impulse required to resolve the collision
        float normalImpulseScalar = (-(1.0f + effectiveRestitution) * speed) / denominator;

        Vector2f normalImpulse = collision.normal * normalImpulseScalar;
        
        RigidBody::applyImpulse(-normalImpulse, radiusA);
        b.applyImpulse(normalImpulse, radiusB);
        
        //Friction
        relativeVelocity = b._velocity + radiusBPerp * b._angularVelocity - _velocity - radiusAPerp * _angularVelocity;
        
        Vector2f tangent = relativeVelocity - (collision.normal * collision.normal.dotProduct(relativeVelocity));
        tangent.normalize();
        
        float radiusAPerpTangent = radiusAPerp.dotProduct(tangent);
        float radiusBPerpTangent = radiusBPerp.dotProduct(tangent);
        
        denominator = ((_inverseMass + b._inverseMass + radiusAPerpTangent * radiusAPerpTangent * _inverseMomentOfInertia + radiusBPerpTangent * radiusBPerpTangent * b._inverseMomentOfInertia) * collision.contactPoints.size());

        //The magnitude of the friction impulse to apply
        float tangentImpulseScalar = -(relativeVelocity.dotProduct(tangent)) / denominator;
        
        if (fabsf(tangentImpulseScalar) < 0.0001f)
            continue;
        
        //If the magnitude of the tangent impulse has broken static friction,
        //we make sure to apply dynamic friction.  If static friction hasn't been broken,
        //we use the magnitude we already solved for
        if (fabsf(tangentImpulseScalar) >= normalImpulseScalar * (_staticFriction + b._staticFriction) * 0.5f)
            tangentImpulseScalar = -normalImpulseScalar * ((_dynamicFriction + b._dynamicFriction) * 0.5f);

        Vector2f tangentImpulse = tangent * tangentImpulseScalar;

        RigidBody::applyImpulse(-tangentImpulse, radiusA);
        b.applyImpulse(tangentImpulse, radiusB);
    }
}

void RigidBody::performCollisionAdjustment(Collision& collision) {
    const float penetrationAllowance = 0.02f;
    const float correctionPercentage = 0.2f;
    
    RigidBody& b = *(collision.bodyB);
    
    Vector2f adjustment = collision.normal * (correctionPercentage * std::max<float>(collision.depth - penetrationAllowance, 0.0f) / (_inverseMass + b._inverseMass));

    if (_inverseMass != 0.0f)
        _position -= adjustment * _inverseMass;
    
    if (b._inverseMass != 0.0f)
        b._position += adjustment * b._inverseMass;
}

float RigidBody::radius() const {
    return _radius;
}

const Vector2f& RigidBody::position() const {
    return _position;
}

float RigidBody::orientation() const {
    return _orientation;
}