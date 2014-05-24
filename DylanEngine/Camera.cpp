#include "Camera.h"

Camera::Camera(const Vector2f& position, float screenWidth, float screenHeight, float mapWidth, float mapHeight) :
_position(position), _screenWidth(screenWidth), _screenHeight(screenHeight), _mapWidth(mapWidth), _mapHeight(mapHeight) {}

void Camera::update() {
    _position += _velocity;
    
    if (_position.x < 0.0f)
        _position.x = 0.0f;
    else if (_position.x + _screenWidth > _mapWidth)
        _position.x = _mapWidth - _screenWidth;
    
    if (_position.y < 0.0f)
        _position.y = 0.0f;
    else if (_position.y + _screenHeight > _mapHeight)
        _position.y = _mapHeight - _screenHeight;
}

void Camera::setVelocity(const Vector2f& velocity) {
    _velocity = velocity;
}

void Camera::setVelocityX(float x) {
    _velocity.x = x;
}

void Camera::setVelocityY(float y) {
    _velocity.y = y;
}

const Vector2f& Camera::position() const {
    return _position;
}