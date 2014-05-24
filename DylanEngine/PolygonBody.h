#ifndef PolygonBody_h
#define PolygonBody_h

#include "RigidBody.h"
#include <vector>
#include "Vector2f.h"

struct Collision;

struct Edge {
    Vector2f vertex, start, end;
};

class PolygonBody : public RigidBody {
private:
    std::vector<Vector2f> _verticesModelSpace, _verticesWorldSpace;
    
private:
    Edge calculateCollisionEdge(const Vector2f& collisionNormal) const;
    std::vector<Vector2f> calculateContactPoints(PolygonBody& polygon, const Vector2f& collisionNormal) const;
    std::vector<Vector2f> clipPoints(const Vector2f& vertexA, const Vector2f& vertexB, const Vector2f& clippingEdge, float offset) const;
    
    float calculateRadiusFromCenterOfMass() const;
    Vector2f calculateCenterOfMass(const std::vector<Vector2f>& vertices) const;
    void updateVerticesWorldSpace();
    
public:
    PolygonBody(const std::vector<Vector2f>& vertices, const Vector2f& position, float mass, float momentOfInertia, float restitution, float staticFriction, float dynamicFriction);
    ~PolygonBody();
    
    void calculateNormals(std::vector<Vector2f>& outputNormals) const;
    
    bool checkCollision(RigidBody& body, Collision& collision);
    bool checkCollision(CircleBody& circle, Collision& collision);
    bool checkCollision(PolygonBody& polygon, Collision& collision);
    
    void integrateVelocity(float timeStep);
    
    const std::vector<Vector2f>& verticesModelSpace() const;
    const std::vector<Vector2f>& verticesWorldSpace() const;
};

#endif
