#include "TestApplication.h"
#include "Timer.h"
#include "PhysicsSimulation.h"
#include "PolygonBody.h"
#include "CircleBody.h"
#include <SFML/Window/Mouse.hpp>

TestApplication::TestApplication() :
_screenWidth(800),
_screenHeight(600),
_camera(Vector2f(0.0f, 0.0f), _screenWidth, _screenHeight, 1600.0f, 1200.0f),
_leftMouseDown(false) {}

TestApplication::~TestApplication() {
    for (size_t i = 0; i < _polygonBodies.size(); ++i) {
        delete _polygons[i];
        delete _polygonBodies[i];
    }
    
    for (size_t i = 0; i < _circleBodies.size(); ++i) {
        delete _circles[i];
        delete _circleBodies[i];
    }
}

bool TestApplication::initialize() {
    _window.create(sf::VideoMode(_screenWidth, _screenHeight, 32), "Dylan's Rigid Body Physics Engine");
    _window.setFramerateLimit(60);
    _window.setVerticalSyncEnabled(false);
    _window.setKeyRepeatEnabled(true);
    _window.setVisible(true);
    _window.setActive(true);
    
    return true;
}

void TestApplication::createGeometry() {
    CircleBody* c = new CircleBody(Vector2f(2.0f, 2.0f), 0.5f, 0.0f, 0.1f, 0.15f, 0.4f, 0.3f, 0.15f);
    c->applyTorque(30.0f);
    _circleBodies.push_back(c);
    _circles.push_back(new sf::CircleShape(c->radius() * 100.0f));
    _circles.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    _circles.back()->setOrigin(c->radius() * 100.0f, c->radius() * 100.0f);
    
    std::vector<Vector2f> vertices;
    
    vertices.push_back(Vector2f(0.0f, 2.0f));
    vertices.push_back(Vector2f(0.0f, 3.0f));
    vertices.push_back(Vector2f(1.0f, 3.0f));
    vertices.push_back(Vector2f(1.0f, 2.0f));
    
    PolygonBody* b = new PolygonBody(vertices, Vector2f(1.5f, 3.5f), 0.1f, 0.2f, 0.3f, 0.4f, 0.25f);
    _polygonBodies.push_back(b);
    b->applyForce(Vector2f(5.2f, -46.9f));
    b->applyTorque(18.0f);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(3.0f, 0.0f));
    vertices.push_back(Vector2f(3.0f, 1.0f));
    vertices.push_back(Vector2f(4.0f, 1.0f));
    vertices.push_back(Vector2f(4.0f, 0.0f));
    
    b = new PolygonBody(vertices, Vector2f(3.5f, 1.5f), 0.2f, 0.09f, 0.59f, 0.9f, 0.4f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(0.0f, 0.0f));
    vertices.push_back(Vector2f(0.0f, 0.4f));
    vertices.push_back(Vector2f(4.0f, 0.4f));
    vertices.push_back(Vector2f(4.0f, 0.0f));
    
    b = new PolygonBody(vertices, Vector2f(9.5f, 1.5f), 0.15f, 0.09f, 0.45f, 0.2f, 0.1f);
    b->applyTorque(17.0f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(0.0f, 4.0f));
    vertices.push_back(Vector2f(2.0f, 4.0f));
    vertices.push_back(Vector2f(1.0f, 3.0f));
    
    b = new PolygonBody(vertices, Vector2f(5.5f, 3.5f), 0.09f, 0.1f, 0.7f, 0.3f, 0.15f);
    _polygonBodies.push_back(b);
    b->applyForce(Vector2f(-44.0f, -27.0f));
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(0.0f, 1.0f));
    vertices.push_back(Vector2f(-0.3f, 1.2f));
    vertices.push_back(Vector2f(-0.5f, 1.7f));
    vertices.push_back(Vector2f(0.0f, 3.0f));
    vertices.push_back(Vector2f(0.8f, 1.9f));
    vertices.push_back(Vector2f(0.6f, 1.0f));
    
    b = new PolygonBody(vertices, Vector2f(3.5f, 0.5f), 0.15f, 0.3f, 0.5f, 0.6f, 0.4f);
    _polygonBodies.push_back(b);
    b->applyTorque(-40.0f);
    b->applyForce(Vector2f(0.0f, -200.0f));
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(1.5f, 1.76f));
    vertices.push_back(Vector2f(1.36f, 0.6f));
    vertices.push_back(Vector2f(2.02f, 0.26f));
    vertices.push_back(Vector2f(4.32f, 0.48f));
    vertices.push_back(Vector2f(5.0f, 2.0f));
    vertices.push_back(Vector2f(4.32f, 3.64f));
    vertices.push_back(Vector2f(2.9f, 3.38f));
    vertices.push_back(Vector2f(1.98f, 2.7f));
    
    b = new PolygonBody(vertices, Vector2f(6.5f, 0.4f), 0.1f, 0.2f, 0.3f, 0.6f, 0.4f);
    _polygonBodies.push_back(b);
    b->applyTorque(-40.0f);
    b->applyForce(Vector2f(0.0f, -300.0f));
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(0.0f, 2.0f));
    vertices.push_back(Vector2f(4.0f, 3.0f));
    vertices.push_back(Vector2f(2.0f, 0.0f));
    
    b = new PolygonBody(vertices, Vector2f(9.5f, 9.5f), 0.0f, 0.0f, 0.8f, 0.1f, 0.05f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    
    
    vertices.push_back(Vector2f(0.5f, 4.3f));
    vertices.push_back(Vector2f(0.5f, 4.7f));
    vertices.push_back(Vector2f(16.0f, 4.7f));
    vertices.push_back(Vector2f(16.0f, 4.3f));
    
    b = new PolygonBody(vertices, Vector2f(3.2f, 5.4f), 0.0f, 0.0f, 0.35f, 0.4f, 0.2f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(rand() % 230, rand() % 230, rand() % 230, 255));
    vertices.clear();
    
    //Top Border
    vertices.push_back(Vector2f(0.0f, 0.0f));
    vertices.push_back(Vector2f(0.0f, 0.5f));
    vertices.push_back(Vector2f(16.0f, 0.5f));
    vertices.push_back(Vector2f(16.0f, 0.0f));
    
    b = new PolygonBody(vertices, Vector2f(8.0f, 0.1f), 0.0f, 0.0f, 0.3f, 0.2f, 0.1f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(150, 0, 120, 255));
    vertices.clear();
    
    //Bottom Border
    vertices.push_back(Vector2f(0.0f, 0.0f));
    vertices.push_back(Vector2f(0.0f, 0.5f));
    vertices.push_back(Vector2f(16.0f, 0.5f));
    vertices.push_back(Vector2f(16.0f, 0.0f));
    
    b = new PolygonBody(vertices, Vector2f(8.0f, 12.0f - 0.1f), 0.0f, 0.0f, 0.3f, 0.2f, 0.1f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(150, 0, 120, 255));
    vertices.clear();
    
    //Left Border
    vertices.push_back(Vector2f(0.0f, 0.21f));
    vertices.push_back(Vector2f(0.0f, 12.0f - 0.51f));
    vertices.push_back(Vector2f(0.2f, 12.0f - 0.51f));
    vertices.push_back(Vector2f(0.2f, 0.21f));
    
    b = new PolygonBody(vertices, Vector2f(0.1f, 6.0f), 0.0f, 0.0f, 0.3f, 0.2f, 0.1f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(150, 0, 120, 255));
    vertices.clear();
    
    //Right Border
    vertices.push_back(Vector2f(0.0f, 0.21f));
    vertices.push_back(Vector2f(0.0f, 12.0f - 0.51f));
    vertices.push_back(Vector2f(0.2f, 12.0f - 0.51f));
    vertices.push_back(Vector2f(0.2f, 0.21f));
    
    b = new PolygonBody(vertices, Vector2f(16.0f - 0.1f, 6.0f), 0.0f, 0.0f, 0.3f, 0.2f, 0.1f);
    _polygonBodies.push_back(b);
    _polygons.push_back(new sf::ConvexShape(vertices.size()));
    _polygons.back()->setFillColor(sf::Color(150, 0, 120, 255));
    vertices.clear();
}

void TestApplication::handleInput() {
    sf::Event event;
    
    while (_window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            _window.close();
        } else if (event.type == sf::Event::KeyReleased) {
            switch (event.key.code) {
                case sf::Keyboard::W:
                    _camera.setVelocityY(0.0f);
                    break;
                case sf::Keyboard::S:
                    _camera.setVelocityY(0.0f);
                    break;
                case sf::Keyboard::A:
                    _camera.setVelocityX(0.0f);
                    break;
                case sf::Keyboard::D:
                    _camera.setVelocityX(0.0f);
                    break;
                default:
                    break;
            }
        } else if (event.type == sf::Event::KeyPressed) {
            switch (event.key.code) {
                case sf::Keyboard::Escape:
                    _window.close();
                    break;
                case sf::Keyboard::W:
                    _camera.setVelocityY(-8.0f);
                    break;
                case sf::Keyboard::S:
                    _camera.setVelocityY(8.0f);
                    break;
                case sf::Keyboard::A:
                    _camera.setVelocityX(-8.0f);
                    break;
                case sf::Keyboard::D:
                    _camera.setVelocityX(8.0f);
                    break;
                default:
                    break;
            }
        } else if ((event.type == sf::Event::MouseButtonPressed) && (event.mouseButton.button == sf::Mouse::Left)) {
            _leftMouseDown = true;
            _previousMousePosition.x = event.mouseButton.x;
            _previousMousePosition.y = event.mouseButton.y;
        } else if (event.type == sf::Event::MouseButtonReleased) {
            _leftMouseDown = false;
        } else if ((event.type == sf::Event::MouseMoved) && _leftMouseDown) {
            _mouseForce.x = event.mouseMove.x - _previousMousePosition.x;
            _mouseForce.y = event.mouseMove.y - _previousMousePosition.y;
            
            _previousMousePosition.x = event.mouseMove.x;
            _previousMousePosition.y = event.mouseMove.y;
        }
    }
}

//Handles checking of collision with mouse and geometry and applying force on collision
void TestApplication::checkMouseCollision() {
    for (size_t i = 0; i < _circleBodies.size(); ++i) {
        CircleBody& body = *_circleBodies[i];
        
        //Check if mouse collides
        if (_leftMouseDown) {
            Vector2f mouse = (_previousMousePosition + _camera.position()) * 0.01f; //Scale mouse position to world coordinates
            
            Vector2f d = mouse - body.position();
            
            if (d.lengthSquared() < body.radius() * body.radius())
                body.applyForce(_mouseForce);
        }
    }
    
    for (size_t i = 0; i < _polygonBodies.size(); ++i) {
        PolygonBody& body = *_polygonBodies[i];
        const std::vector<Vector2f>& verts = body.verticesWorldSpace();
        
        //Check if the polygon and mouse collide
        if (_leftMouseDown) {
            Vector2f mouse = (_previousMousePosition + _camera.position()) * 0.01f; //Scale mouse position to world coordinates
            
            Vector2f d = mouse - body.position();
            
            if (d.lengthSquared() < body.radius() * body.radius()) {
                bool collides = false;
                
                for (int j = 0, k = verts.size() - 1; j < verts.size(); k = j++) {
                    if ( ((verts[j].y > mouse.y) != (verts[k].y > mouse.y)) && (mouse.x < (verts[k].x - verts[j].x) * (mouse.y - verts[j].y) / (verts[k].y - verts[j].y) + verts[j].x))
                        collides = !collides;
                }
                
                if (collides)
                    body.applyForce(_mouseForce);
            }
        }
    }
}

void TestApplication::renderGeometry() {
    for (size_t i = 0; i < _circleBodies.size(); ++i) {
        CircleBody& body = *_circleBodies[i];
        
        _circles[i]->setPosition(body.position().x * 100.0f - _camera.position().x, body.position().y * 100.0f - _camera.position().y);
        _window.draw(*_circles[i]);
        
        float cosOrientation = cosf(body.orientation());
        float sinOrientation = sinf(body.orientation());
        
        sf::VertexArray line(sf::LinesStrip, 2);
        line[0].position = sf::Vector2f(body.position().x * 100.0f - _camera.position().x, body.position().y * 100.0f - _camera.position().y);
        line[1].position = sf::Vector2f(((0.0f * cosOrientation - 0.5f * sinOrientation) + body.position().x) * 100.0f - _camera.position().x, ((-0.5f * cosOrientation - 0.0f * sinOrientation) + body.position().y) * 100.0f - _camera.position().y);
        
        _window.draw(line);
    }
    
    for (size_t i = 0; i < _polygonBodies.size(); ++i) {
        PolygonBody& body = *_polygonBodies[i];
        const std::vector<Vector2f>& verts = body.verticesWorldSpace();
        
        for (size_t j = 0; j < verts.size(); ++j)
            _polygons[i]->setPoint(j, sf::Vector2f(verts[j].x * 100.0f - _camera.position().x, verts[j].y * 100.0f - _camera.position().y));
        
        _window.draw(*_polygons[i]);
    }
    
    _window.display();
}

int TestApplication::run() {
    if (!initialize())
        return -1;
    
    PhysicsSimulation simulation(Vector2f(0.0f, 10.0f), 10);

    createGeometry();
    
    for (std::vector<CircleBody*>::const_iterator i = _circleBodies.begin(); i != _circleBodies.end(); ++i)
        simulation.addBody(*i);
    
    for (std::vector<PolygonBody*>::const_iterator i = _polygonBodies.begin(); i != _polygonBodies.end(); ++i)
        simulation.addBody(*i);

    Timer _timer(1000.0f / 60); //60 updates per second
    _timer.start();
    
    while (_window.isOpen()) {
        handleInput();
        
        if (_mouseForce.x < -100.0f)
            _mouseForce.x = -100.0f;
        else if (_mouseForce.x > 100.0f)
            _mouseForce.x = 100.0f;
        
        if (_mouseForce.y < -100.0f)
            _mouseForce.y = -100.0f;
        else if (_mouseForce.y > 100.0f)
            _mouseForce.y = 100.0f;

        _timer.update();
        
        while (_timer.checkTimeStep())
            simulation.update(_timer.timeStep() * 0.001f);
        
        _camera.update();
        
        _window.clear();
        
        checkMouseCollision();
        
        renderGeometry();
    }
    
    return 0;
}