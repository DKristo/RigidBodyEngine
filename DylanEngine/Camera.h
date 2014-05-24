#ifndef Camera_h
#define Camera_h

#include "Vector2f.h"

class Camera {
private:
    Vector2f _position, _velocity;
    float _screenWidth, _screenHeight;
    float _mapWidth, _mapHeight;

public:
    Camera(const Vector2f& position, float screenWidth, float screenHeight, float mapWidth, float mapHeight);
    ~Camera() {}
    
    void update();
    
    void setVelocityX(float x);
    void setVelocityY(float y);
    void setVelocity(const Vector2f& velocity);
    
    const Vector2f& position() const;
};

#endif
