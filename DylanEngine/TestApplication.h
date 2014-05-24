#ifndef TestApplication_h
#define TestApplication_h

#include <SFML/Graphics.hpp>
#include <vector>
#include "Camera.h"

class PolygonBody;
class CircleBody;

class TestApplication {
private:
    const unsigned int _screenWidth, _screenHeight;
    sf::RenderWindow _window;
    
    Camera _camera;
    
    bool _leftMouseDown;
    Vector2f _previousMousePosition, _mouseForce;
    
    std::vector<PolygonBody*> _polygonBodies;
    std::vector<sf::ConvexShape*> _polygons;
    
    std::vector<CircleBody*> _circleBodies;
    std::vector<sf::CircleShape*> _circles;
    
private:
    bool initialize();
    void createGeometry();
    void handleInput();
    void checkMouseCollision();
    void renderGeometry();
    
public:
    TestApplication();
    ~TestApplication();
    
    int run();
};

#endif
