#include "CircleBody.h"
#include <algorithm>
#include "Collision.h"
#include "PolygonBody.h"

CircleBody::CircleBody(const Vector2f& position, float radius, float orientation, float mass, float momentOfInertia, float restitution, float staticFriction, float dynamicFriction) :
RigidBody(position, orientation, mass, momentOfInertia, restitution, staticFriction, dynamicFriction) {
    _radius = radius;
}

CircleBody::~CircleBody() {}

bool CircleBody::checkCollision(RigidBody& body, Collision& collision) {
    return body.checkCollision(*this, collision);
}

bool CircleBody::checkCollision(CircleBody& circle, Collision& collision) {
    Vector2f d = circle._position - _position;
    
    float distanceSquared = d.lengthSquared();
    
    float radiusSum = _radius + circle._radius;
    
    if (distanceSquared < radiusSum * radiusSum) {
        collision.bodyA = this;
        collision.bodyB = &circle;
        
        collision.normal = d;
        collision.normal.normalize();
        
        collision.depth = radiusSum - sqrtf(distanceSquared);
        
        collision.contactPoints.push_back(_position - collision.normal * (_radius - collision.depth));
        
        return true;
    }
    
    return false;
}

bool CircleBody::checkCollision(PolygonBody& polygon, Collision& collision) {
    return polygon.checkCollision(*this, collision);
}