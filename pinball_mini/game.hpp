//
//  game.hpp
//  sfml_rigidBody
//
//  Created by 이재현 on 2020/10/04.
//

#ifndef game_hpp
#define game_hpp

#include <box2d/box2d.h>
#include "geometry.cpp"

struct b2UserData{
    b2Color color;
    
    bool is_solid;
};



class Game{
    
    
public:
    
    Game();
    ~Game();
    
    
    b2World* world;
    b2Vec2 Gravity;
    
    b2Body* m_ball;
    b2Body* ground = NULL;
    b2Body* m_obs_rotate;
    b2Body* m_obs_nail[7];
    b2Body* m_obs_mov_rect[3];
    b2Body* leftFlipper;
    b2Body* rightFlipper;
    b2Body* m_obs_freeball;
    b2Body* m_obs_blackhole;
    b2Body* m_water;
    
//    Myb2ContactListener my_listner;
    
    
    
//    b2BodyUserData ground_data;
//    b2BodyUserData m_ball_data;
//    b2BodyUserData m_flipper_data;
//    b2BodyUserData m_obs_rotate_data;
//    b2BodyUserData m_obs_nail_data;
//    b2BodyUserData m_obs_freeball_data;
//    b2BodyUserData m_obs_blackhole_data;
//    b2BodyUserData m_obs_mov_rect_data;
//    b2BodyUserData m_water_data;
    
    b2RevoluteJoint* m_leftJoint;
    b2RevoluteJoint* m_rightJoint;
    
    b2Vec2 m_obs_mov_rect_speed;
    
    
    
   
    
   
    
//    b2GLDraw fooDrawInstance;
    
    float timeStep=1.f/60;
    int velocityIter=10;
    int positionIter=8;
    
    const float MAP_WIDTH=800.f;
    const float MAP_HEIGHT=700.f;
    const int WINDOW_WIDTH=3000;
    const int WINDOW_HEIGHT=1500;
    float SCALE=30.f;
    int BodyCount=0;
    bool m_button = false;
    
    void createMap();
    void createBox(float,float);
    void render();
    void step();
    
    void doFlip(int);
    void checkReverse();
    void checkBlackHole();
    
    void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color,b2Vec2& bodyPos,float rot);
    
    void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color,b2Vec2& bodyPos,float rot);
    
    void DrawCircle(const b2Vec2& center, float radius, const b2Color& color);
    
    void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color);
    
    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color);
    
//    void DrawTransform(const b2Transform& xf);
    
    void DrawPoint(const b2Vec2& p, float size, const b2Color& color);
    
    void checkBuoyancy();
    
//    void DrawString(int x, int y, const char* string, ...);
//
//    void DrawAABB(b2AABB* aabb, const b2Color& color);



};

#endif /* game_hpp */
