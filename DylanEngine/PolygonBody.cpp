#include "PolygonBody.h"
#include <algorithm>
#include "Collision.h"
#include "CircleBody.h"

PolygonBody::PolygonBody(const std::vector<Vector2f>& vertices, const Vector2f& position, float mass, float momentOfInertia, float restitution, float staticFriction, float dynamicFriction) :
RigidBody(position, 0.0f, mass, momentOfInertia, restitution, staticFriction, dynamicFriction) {
    Vector2f absolutePosition = calculateCenterOfMass(vertices);
    
    for (std::vector<Vector2f>::const_iterator i = vertices.begin(); i != vertices.end(); ++i)
        _verticesModelSpace.push_back((*i) - absolutePosition);
    
    _verticesWorldSpace.resize(_verticesModelSpace.size());
    updateVerticesWorldSpace();
    
    _radius = calculateRadiusFromCenterOfMass();
}

PolygonBody::~PolygonBody() {}

float PolygonBody::calculateRadiusFromCenterOfMass() const {
    float max = 0.0f;
    
    for (std::vector<Vector2f>::const_iterator i = _verticesWorldSpace.begin(); i != _verticesWorldSpace.end(); ++i)
        max = std::max(max, ((*i) - _position).lengthSquared());
    
    return sqrtf(max);
}

//Calculates the centroid of our polygon, assuming uniform density
Vector2f PolygonBody::calculateCenterOfMass(const std::vector<Vector2f>& vertices) const {
    Vector2f result(0.0f, 0.0f);
    float areaFactor = 0.0f;
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        const Vector2f& a = vertices[i];
        const Vector2f& b = vertices[(i + 1) % vertices.size()];
        
        float temp = a.x * b.y - b.x * a.y;
        
        areaFactor += temp;
        
        result.x += temp * (a.x + b.x);
        result.y += temp * (a.y + b.y);
    }

    areaFactor *= 3.0f;

    result.x /= areaFactor;
    result.y /= areaFactor;
        
    return result;
}

void PolygonBody::calculateNormals(std::vector<Vector2f>& outputNormals) const {
    for (size_t i = 0; i < _verticesWorldSpace.size(); ++i) {
        Vector2f edge = _verticesWorldSpace[i] - _verticesWorldSpace[(i + 1) % _verticesWorldSpace.size()];
        edge.normalize();
        
        outputNormals.push_back(edge.crossProduct(-1.0f));
    }
}

bool PolygonBody::checkCollision(RigidBody& body, Collision& collision) {
    return body.checkCollision(*this, collision);
}

//We use a modified Separating Axis Test to test for polygon-circle collisions
bool PolygonBody::checkCollision(CircleBody& circle, Collision& collision) {
    float radiusSum = _radius + circle.radius();
    
    if ((circle.position() - _position).lengthSquared() > radiusSum * radiusSum)
        return false;
    
    std::vector<Vector2f> axes;
    this->calculateNormals(axes);
    
    //We need the axis from the center of the circle to the closest vertex on the polygon
    Vector2f additionalAxis = _verticesWorldSpace[0] - circle.position();
    
    for (size_t i = 1; i < _verticesWorldSpace.size(); ++i) {
        Vector2f v = _verticesWorldSpace[i] - circle.position();
        
        if (v.lengthSquared() < additionalAxis.lengthSquared())
            additionalAxis = v;
    }
    
    additionalAxis.normalize();
    axes.push_back(additionalAxis);
    
    float minOverlap = std::numeric_limits<float>::max();
    Vector2f minOverlapAxis;
    
    for (size_t i = 0; i < axes.size(); ++i) {
        Vector2f& axis = axes[i];
        
        float projection = axis.dotProduct(_verticesWorldSpace.front());
        float aMinProjection = projection;
        float aMaxProjection = projection;
        
        for (size_t j = 1; j < _verticesWorldSpace.size(); ++j) {
            projection = axis.dotProduct(_verticesWorldSpace[j]);
            
            aMinProjection = std::min(aMinProjection, projection);
            aMaxProjection = std::max(aMaxProjection, projection);
        }
        
        projection = axis.dotProduct(circle.position());
        float bMinProjection = projection - circle.radius();
        float bMaxProjection = projection + circle.radius();
        
        if (bMinProjection > bMaxProjection)
            std::swap(bMinProjection, bMaxProjection);
        
        if ((aMaxProjection < bMinProjection) || (bMaxProjection < aMinProjection))
            return false; //This axis has no overlap, a collision is not possible
        
        float lowerOverlap = aMaxProjection - bMinProjection;
        float upperOverlap = bMaxProjection - aMinProjection;
        
        if (lowerOverlap < minOverlap) {
            minOverlap = lowerOverlap;
            minOverlapAxis = axis;
        }
        
        if (upperOverlap < minOverlap) {
            minOverlap = upperOverlap;
            minOverlapAxis = -axis;
        }
    }

    collision.bodyA = this;
    collision.bodyB = &circle;
    collision.normal = minOverlapAxis;
    collision.depth = minOverlap;
    collision.contactPoints.push_back(circle.position() - collision.normal * (circle.radius() - collision.depth));

    return true;
}

//We use the Separating Axis Test to check for a polygon-polygon collision
bool PolygonBody::checkCollision(PolygonBody& polygon, Collision& collision) {
    float combinedRadius = _radius + polygon._radius;
    
    //Do a quick bounding circle check first
    if ((polygon._position - _position).lengthSquared() > combinedRadius * combinedRadius)
        return false;

    std::vector<Vector2f> axes;
    this->calculateNormals(axes);
    polygon.calculateNormals(axes);
    
    float minOverlap = std::numeric_limits<float>::max();
    Vector2f minOverlapAxis;
    
    for (size_t i = 0; i < axes.size(); ++i) {
        const Vector2f& axis = axes[i];
        
        float projection = axis.dotProduct(_verticesWorldSpace.front());
        float aMinProjection = projection;
        float aMaxProjection = projection;
        
        for (size_t j = 1; j < _verticesWorldSpace.size(); ++j) {
            projection = axis.dotProduct(_verticesWorldSpace[j]);
            
            aMinProjection = std::min(aMinProjection, projection);
            aMaxProjection = std::max(aMaxProjection, projection);
        }
        
        projection = axis.dotProduct(polygon._verticesWorldSpace.front());
        float bMinProjection = projection;
        float bMaxProjection = projection;
        
        for (size_t j = 1; j < polygon._verticesWorldSpace.size(); ++j) {
            projection = axis.dotProduct(polygon._verticesWorldSpace[j]);
            
            bMinProjection = std::min(bMinProjection, projection);
            bMaxProjection = std::max(bMaxProjection, projection);
        }

        if ((aMaxProjection < bMinProjection) || (bMaxProjection < aMinProjection))
            return false; //This axis has no overlap, a collision is not possible
        
        float lowerOverlap = aMaxProjection - bMinProjection;
        float upperOverlap = bMaxProjection - aMinProjection;
        
        if (lowerOverlap < minOverlap) {
            minOverlap = lowerOverlap;
            minOverlapAxis = axis;
        }
        
        if (upperOverlap < minOverlap) {
            minOverlap = upperOverlap;
            minOverlapAxis = -axis;
        }
    }

    //A collision has occured
    //minOverlapAxis is our collision normal, minOverlap is the penetration depth
    collision.contactPoints = calculateContactPoints(polygon, minOverlapAxis);
    
    if (collision.contactPoints.size() == 0)
        return false;
    
    collision.bodyA = this;
    collision.bodyB = &polygon;
    collision.normal = minOverlapAxis;
    collision.depth = minOverlap;

    return true;
}

//Calculates either 1 or 2 contact points for a collision depending on
//how the polygons overlap
std::vector<Vector2f> PolygonBody::calculateContactPoints(PolygonBody& polygon, const Vector2f& collisionNormal) const {
    Edge edgeA = calculateCollisionEdge(collisionNormal);
    Edge edgeB = polygon.calculateCollisionEdge(-collisionNormal);
    
    const Edge* referenceEdge = 0;
    const Edge* incidentEdge = 0;
    
    float dotA = (edgeA.end - edgeA.start).dotProduct(collisionNormal);
    float dotB = (edgeB.end - edgeB.start).dotProduct(collisionNormal);
    
    //The reference edge is the most perpendicular to the collision normal (dot product is closest to zero)
    if (fabsf(dotA) <= fabsf(dotB)) {
        referenceEdge = &edgeA;
        incidentEdge = &edgeB;
    } else {
        referenceEdge = &edgeB;
        incidentEdge = &edgeA;
    }
    
    Vector2f referenceVector = referenceEdge->end - referenceEdge->start;
    referenceVector.normalize();

    std::vector<Vector2f> clippedPoints = clipPoints(incidentEdge->start, incidentEdge->end, referenceVector, referenceVector.dotProduct(referenceEdge->start));

    if (clippedPoints.size() < 2)
        return std::vector<Vector2f>(); //something went wrong, return an empty vector
    
    clippedPoints = clipPoints(clippedPoints[0], clippedPoints[1], -referenceVector, -(referenceVector.dotProduct(referenceEdge->end)));
    
    if (clippedPoints.size() < 2)
        return std::vector<Vector2f>(); //something went wrong, return an empty vector

    Vector2f referenceNormal = referenceVector.crossProduct(-1.0f);
    
    float max = referenceNormal.dotProduct(referenceEdge->vertex);
    
    if (referenceNormal.dotProduct(clippedPoints[1]) > max)
        clippedPoints.erase(clippedPoints.begin() + 1);
    
    if (referenceNormal.dotProduct(clippedPoints[0]) > max)
        clippedPoints.erase(clippedPoints.begin());
    
    return clippedPoints;
}

//Calculates the closest edge involved in the collision
Edge PolygonBody::calculateCollisionEdge(const Vector2f& collisionNormal) const {
    float maxProjection = collisionNormal.dotProduct(_verticesWorldSpace[0]);
    int vertexIndex = 0;
        
    //We are looking for the furthest projected vertex
    for (size_t i = 1; i < _verticesWorldSpace.size(); ++i) {
        float projection = collisionNormal.dotProduct(_verticesWorldSpace[i]);
        
        if (projection > maxProjection) {
            maxProjection = projection;
            vertexIndex = i;
        }
    }

    Edge edge;
    edge.vertex = _verticesWorldSpace[vertexIndex];
    
    const Vector2f& previousVertex = _verticesWorldSpace[(vertexIndex - 1) % _verticesWorldSpace.size()];
    const Vector2f& nextVertex = _verticesWorldSpace[(vertexIndex + 1) % _verticesWorldSpace.size()];
    
    Vector2f edgeA = edge.vertex - nextVertex;
    Vector2f edgeB = edge.vertex - previousVertex;
    
    //We take the dot products and compare them to find the most perpendicular edge
    if (fabsf(edgeA.dotProduct(collisionNormal)) <= fabsf(edgeB.dotProduct(collisionNormal))) {
        edge.start = edge.vertex;
        edge.end = nextVertex;
    } else {
        edge.start = previousVertex;
        edge.end = edge.vertex;
    }
    
    return edge;
}

//This is just a simple clipping function for clipping a line(vertexA, vertexB)
//with respect to another line (clippingEdge, offset)
std::vector<Vector2f> PolygonBody::clipPoints(const Vector2f& vertexA, const Vector2f& vertexB, const Vector2f& clippingEdge, float offset) const {
    std::vector<Vector2f> clippedPoints;
    
    //We start by checking if each vertex is within the clipping region defined by the clippingEdge and offset along it
    //If a vertex is outside of the clipping region, we add it to our list of clipped points
    float distanceA = clippingEdge.dotProduct(vertexA) - offset;
    
    if (distanceA >= 0.0f)
        clippedPoints.push_back(vertexA);
    
    float distanceB = clippingEdge.dotProduct(vertexB) - offset;
    
    if (distanceB >= 0.0f)
        clippedPoints.push_back(vertexB);
    
    //If the points are on opposing sides of the clipping plane,
    //we need to compute a new point at the clipping limit
    if (distanceA * distanceB < 0.0f) {
        Vector2f edge = vertexB - vertexA;
        
        float d = distanceA / (distanceA - distanceB);
        
        edge *= d;
        edge += vertexA;
        
        clippedPoints.push_back(edge);
    }
    
    return clippedPoints;
}

void PolygonBody::integrateVelocity(float timeStep) {
    RigidBody::integrateVelocity(timeStep);
    
    if ((_inverseMass != 0.0f) && (_inverseMomentOfInertia != 0.0f))
        updateVerticesWorldSpace();
}

//Transforms the model coordinates into world space coordinates
//using the orientation and position
void PolygonBody::updateVerticesWorldSpace() {
    float sinOrientation = sinf(_orientation);
    float cosOrientation = cosf(_orientation);
    
    for (size_t i = 0; i < _verticesModelSpace.size(); ++i) {
        const Vector2f& v = _verticesModelSpace[i];
        Vector2f rotatedVertex(v.x * cosOrientation + v.y * sinOrientation, v.y * cosOrientation - v.x * sinOrientation);

        _verticesWorldSpace[i] = rotatedVertex + _position;
    }
}

const std::vector<Vector2f>& PolygonBody::verticesModelSpace() const {
    return _verticesModelSpace;
}

const std::vector<Vector2f>& PolygonBody::verticesWorldSpace() const {
    return _verticesWorldSpace;
}