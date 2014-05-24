#ifndef CircleBody_h
#define CircleBody_h

#include "RigidBody.h"

struct Collision;

class CircleBody : public RigidBody {
public:
    CircleBody(const Vector2f& position, float radius, float orientation, float mass, float momentOfInertia, float restitution, float staticFriction, float dynamicFriction);
    ~CircleBody();

    bool checkCollision(RigidBody& body, Collision& collision);
    bool checkCollision(CircleBody& circle, Collision& collision);
    bool checkCollision(PolygonBody& polygon, Collision& collision);
};

#endif
