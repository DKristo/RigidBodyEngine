#ifndef Vector2f_h
#define Vector2f_h

#include <cmath>
#include <limits>

class Vector2f {
public:
    float x, y;
    
    Vector2f();
    Vector2f(float x, float y);
    ~Vector2f() {}
    
    Vector2f& operator+=(const Vector2f& vector);
    Vector2f& operator-=(const Vector2f& vector);
    Vector2f& operator*=(float scalar);
    
    Vector2f operator+(const Vector2f& vector) const;
    Vector2f operator-(const Vector2f& vector) const;
    Vector2f operator*(float scalar) const;
    
    Vector2f operator-() const;
    
    void normalize();
    
    float dotProduct(const Vector2f& vector) const;
    float length() const;
    float lengthSquared() const;
    
    float crossProduct(const Vector2f& vector) const;
    Vector2f crossProduct(float scalar) const;
};

inline Vector2f::Vector2f() : x(0.0f), y(0.0f) {}

inline Vector2f::Vector2f(float x, float y) : x(x), y(y) {}

inline Vector2f& Vector2f::operator+=(const Vector2f& vector) {
    x += vector.x;
    y += vector.y;
    
    return *this;
}

inline Vector2f& Vector2f::operator-=(const Vector2f& vector) {
    x -= vector.x;
    y -= vector.y;
    
    return *this;
}

inline Vector2f Vector2f::operator*(float scalar) const {
    return Vector2f(x * scalar, y * scalar);
}

inline Vector2f& Vector2f::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    
    return *this;
}

inline Vector2f Vector2f::operator+(const Vector2f& vector) const {
    return Vector2f(x + vector.x, y + vector.y);
}

inline Vector2f Vector2f::operator-(const Vector2f& vector) const {
    return Vector2f(x - vector.x, y - vector.y);
}

inline Vector2f Vector2f::operator-() const {
    return Vector2f(-x, -y);
}

inline void Vector2f::normalize() {
    float magnitudeSquared = lengthSquared();
    
    if (magnitudeSquared > std::numeric_limits<float>::epsilon()) {
        float magnitude = sqrtf(magnitudeSquared);
        
        x /= magnitude;
        y /= magnitude;
    }
}

inline float Vector2f::dotProduct(const Vector2f& vector) const {
    return x*vector.x + y*vector.y;
}

inline float Vector2f::length() const {
    return sqrtf(x*x + y*y);
}

inline float Vector2f::lengthSquared() const {
    return x*x + y*y;
}

inline float Vector2f::crossProduct(const Vector2f& vector) const {
    return x * vector.y - y * vector.x;
}

inline Vector2f Vector2f::crossProduct(float scalar) const {
    return Vector2f(scalar * y, -scalar * x);
}

#endif