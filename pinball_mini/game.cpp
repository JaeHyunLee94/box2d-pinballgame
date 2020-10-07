
//
//
//  game.cpp
//  sfml_rigidBody
//
//  Created by 이재현 on 2020/10/04.
//

#include "game.hpp"
#include <box2d/box2d.h>
#include <iostream>

#include "signal.h"
#include <GL/glew.h>
#include <OpenGL/OpenGL.h>

#include <GLUT/GLUT.h>
#include "geometry.cpp"

//constructor
Game::Game(){
    
    
    Gravity=b2Vec2(0,-9.8);
    m_obs_mov_rect_speed.Set(3, 0);
    world=new b2World(Gravity);
//    world->SetContactListener(&my_listner);
    
}

Game::~Game(){
    delete world;
}

void Game::createMap(){
    
    
    
    
    b2Body* ground = NULL;
    {
        b2BodyDef bd;
        ground = world->CreateBody(&bd);
        
        b2Vec2 vs[18];
        vs[0].Set(15, 25);
        vs[1].Set(15, 20);
        vs[2].Set(20, 20);
        vs[3].Set(25, 14);
        vs[4].Set(25, -25);
        vs[5].Set(20, -25);
        vs[6].Set(20, 14);
        vs[7].Set(17, 15);
        vs[8].Set(15, 15);
        vs[9].Set(15, -10);
        
        vs[10].Set(3, -22);
        
        vs[11].Set(18, -22);
        vs[12].Set(18, -28);
        vs[13].Set(-18, -28);
        vs[14].Set(-18, -22);
        vs[15].Set(-3, -22);
        
        
        vs[16].Set(-15, -10);
        vs[17].Set(-15, 25);
        
        
        
        
        b2ChainShape loop;
        loop.CreateLoop(vs, 18);
        b2FixtureDef fd;
        fd.restitution=0.2f;
        fd.shape = &loop;
        fd.density = 0.0f;
        ground->CreateFixture(&fd);
    }
    // Flippers
    {
        
        
        b2Vec2 p1(-5,-20), p2(5,-20);
        
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        
        bd.position = p1;
        leftFlipper = world->CreateBody(&bd);
        
        bd.position = p2;
        rightFlipper = world->CreateBody(&bd);
        
        b2PolygonShape box;
        box.SetAsBox(4, 0.1);
        
        b2FixtureDef fd;
        fd.shape = &box;
        fd.restitution=1.0f;
        fd.density = 2.f;
        
        leftFlipper->CreateFixture(&fd);
        rightFlipper->CreateFixture(&fd);
        
        b2RevoluteJointDef jd;
        jd.bodyA = ground;
        jd.localAnchorB.SetZero();
        jd.enableMotor = true;
        jd.maxMotorTorque = 5000.0f;
        jd.enableLimit = true;
        
        jd.motorSpeed = 0.0f;
        
        jd.localAnchorA=p1;
        jd.bodyB = leftFlipper;
        jd.lowerAngle = -5.0f * b2_pi / 180.0f;
        jd.upperAngle = 30.0f * b2_pi / 180.0f;
        m_leftJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
        
        jd.motorSpeed = 0.0f;
        
        jd.localAnchorA=p2;
        jd.bodyB = rightFlipper;
        jd.lowerAngle = -30.0f * b2_pi / 180.0f;
        jd.upperAngle = 5.0f * b2_pi / 180.0f;
        m_rightJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
    }
    //ball
    {
        
        
        b2BodyDef bd;
        bd.position.Set(23, -20);
        bd.type = b2_dynamicBody;
        bd.bullet = true;
        
        m_ball = world->CreateBody(&bd);
        
        b2CircleShape shape;
        shape.m_radius = 0.5;
        
        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 1.0f;
        fd.friction=0.3f;
        
        m_ball->CreateFixture(&fd);
        m_ball->ApplyLinearImpulse(b2Vec2(0,100), m_ball->GetLocalCenter(), true);
        m_ball->SetLinearDamping(0.1f);
    }
    
    //rotate obstacle
    {
        b2BodyDef bd;
        bd.position.Set(8, 11);
        bd.type=b2_kinematicBody;
        
        m_obs_rotate=world->CreateBody(&bd);
        b2PolygonShape shape;
        shape.SetAsBox(3, 0.4);
        
        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 1.0f;
        m_obs_rotate->CreateFixture(&fd);
        m_obs_rotate->SetAngularVelocity(5);
        
        
    }
    
    //freeball
    {
        b2BodyDef bd;
        bd.position.Set(-4, 6);
        bd.type=b2_dynamicBody;
        
        m_obs_freeball=world->CreateBody(&bd);
        b2CircleShape shape;
        shape.m_radius=1.0f;
        
        
        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 0.2f;
        fd.restitution=0.5;
        
        m_obs_freeball->CreateFixture(&fd);
        
        m_obs_freeball->SetGravityScale(0);
        m_obs_freeball->SetLinearDamping(0.3f);
        
        
        
    }
    
    //nail
    {
        b2BodyDef bd;
        bd.type=b2_kinematicBody;
        
        bd.position.Set(-11, 20);
        m_obs_nail[0]=world->CreateBody(&bd);
        bd.position.Set(-9, 20);
        m_obs_nail[1]=world->CreateBody(&bd);
        bd.position.Set(-12, 18);
        m_obs_nail[2]=world->CreateBody(&bd);
        bd.position.Set(-10, 18);
        m_obs_nail[3]=world->CreateBody(&bd);
        bd.position.Set(-8, 18);
        m_obs_nail[4]=world->CreateBody(&bd);
        bd.position.Set(-11, 16);
        m_obs_nail[5]=world->CreateBody(&bd);
        bd.position.Set(-9, 16);
        m_obs_nail[6]=world->CreateBody(&bd);
        
        
        b2CircleShape shape;
        shape.m_radius=0.2f;
        
        
        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 0.2f;
        fd.restitution=1.0f;
        
        
        for(int i=0;i<7;i++){
            m_obs_nail[i]->CreateFixture(&fd);
            
        }
        
        //mov rect
        {
            b2BodyDef bd;
            bd.type=b2_kinematicBody;
            
            bd.position.Set(-4, 2);
            m_obs_mov_rect[0]=world->CreateBody(&bd);
            bd.position.Set(0, 0);
            m_obs_mov_rect[1]=world->CreateBody(&bd);
            bd.position.Set(-4, -2);
            m_obs_mov_rect[2]=world->CreateBody(&bd);
            
            
            b2PolygonShape shape;
            shape.SetAsBox(2, 0.2);
            
            b2FixtureDef fd;
            fd.restitution=0.8f;
            fd.shape = &shape;
            fd.density = 0.2f;
            
            for(int i=0;i<3;i++){
                m_obs_mov_rect[i]->CreateFixture(&fd);
            }
            m_obs_mov_rect[0]->SetLinearVelocity(m_obs_mov_rect_speed);
            m_obs_mov_rect[1]->SetLinearVelocity(-m_obs_mov_rect_speed);
            m_obs_mov_rect[2]->SetLinearVelocity(m_obs_mov_rect_speed);
            
            
            
            
        }
        
        //blackhole
        {
            b2BodyDef bd;
            bd.position.Set(5, 5);
            bd.type=b2_staticBody;
            
            m_obs_blackhole=world->CreateBody(&bd);
            
            b2CircleShape shape;
            shape.m_radius=2.0f;
            
            
            b2FixtureDef fd;
            fd.isSensor=true;
            fd.shape=&shape;
            fd.density=1.0f;
            
            m_obs_blackhole->CreateFixture(&fd);
            
        }
       
        {
            b2BodyDef bd;
            bd.position.Set(0, -26);
            bd.type=b2_staticBody;
            m_water=world->CreateBody(&bd);
            
            b2PolygonShape shape;
            shape.SetAsBox(18, 2);
            
            
            b2FixtureDef fd;
            fd.isSensor=true;
            fd.shape=&shape;
            fd.density=4.0f;
            
            m_water->CreateFixture(&fd);
            
        }
        
        
        
        
    }
}
void Game::createBox(float MouseX,float MouseY){
    b2BodyDef BodyDef;
    std::cout <<"create box!!\n";
    
    
    BodyDef.position = b2Vec2(MouseX/SCALE, MouseY/SCALE);
    BodyDef.type = b2_dynamicBody;
    b2Body* Body = world->CreateBody(&BodyDef);
    
    b2PolygonShape Shape;
    Shape.SetAsBox((32.f / 2) / SCALE, (32.f / 2) / SCALE);
    b2FixtureDef FixtureDef;
    FixtureDef.density = 1.f;
    FixtureDef.friction = 0.7f;
    FixtureDef.shape = &Shape;
    Body->CreateFixture(&FixtureDef);
}

void Game::render(){
    
    for (b2Body* BodyIterator = world->GetBodyList(); BodyIterator != 0; BodyIterator = BodyIterator->GetNext())
    {
        b2Vec2 bodyPos=BodyIterator->GetPosition();
        
        float bodyrot=(BodyIterator->GetAngle())*180/b2_pi;
        
        for (b2Fixture* FixtureIterator = BodyIterator->GetFixtureList(); FixtureIterator != 0; FixtureIterator = FixtureIterator->GetNext()){
            
            if(FixtureIterator->GetType()==b2Shape::e_circle){
                //                std::cout << "circle pos: "<<bodyPos.x << " "<< bodyPos.y<<"\n";
                b2CircleShape* circleShape = (b2CircleShape*)FixtureIterator->GetShape();
                DrawSolidCircle(bodyPos, circleShape->m_radius, b2Vec2(0,1),b2Color(1,0,0));
                
                
            }else if(FixtureIterator->GetType()==b2Shape::e_edge){
                b2EdgeShape* edgeShape = (b2EdgeShape*)FixtureIterator->GetShape();
                
                DrawSegment(edgeShape->m_vertex1, edgeShape->m_vertex2, b2Color(0,0,1));
                
                
            }else if(FixtureIterator->GetType()==b2Shape::e_polygon){
                
                b2PolygonShape* polygonShape = (b2PolygonShape*)FixtureIterator->GetShape();
                b2Color clr(0,1,0);
                if(FixtureIterator->IsSensor()) clr.Set(0.5, 0.5, 1);
                DrawSolidPolygon(polygonShape->m_vertices, polygonShape->m_count, clr,bodyPos,bodyrot);
            }else{
                
                b2ChainShape* chainShape = (b2ChainShape*)FixtureIterator->GetShape();
                DrawPolygon(chainShape->m_vertices, chainShape->m_count, b2Color(0,1,0),bodyPos,bodyrot);
                
                
            }
            
        }
        
        
        
    }
    
    
}

void Game::step(){
    
    checkReverse();
    checkBlackHole();
    checkBuoyancy();
    world->Step(timeStep, velocityIter, positionIter);
    
}

void Game::doFlip(int signal){
    
    
    switch (signal) {
            
        case GameSignal::DO_FLIP:
            m_leftJoint->SetMotorSpeed(100.0f);
            m_rightJoint->SetMotorSpeed(-100.0f);
            break;
            
        case GameSignal::DO_NOT_FLIP:
            m_leftJoint->SetMotorSpeed(-70.0f);
            m_rightJoint->SetMotorSpeed(70.0f);
            break;
            
        default:
            break;
    }
    
}

void Game::checkReverse(){
    
    if(m_obs_mov_rect[0]->GetPosition().x <-8 || m_obs_mov_rect[0]->GetPosition().x >3 ){
        m_obs_mov_rect_speed=-m_obs_mov_rect_speed;
        for(int i=0;i<3;i++) m_obs_mov_rect[i]->SetLinearVelocity(-m_obs_mov_rect[i]->GetLinearVelocity());
    }
    
    
}

void Game::checkBlackHole(){
    
    b2Vec2 nowPos=m_ball->GetPosition();
    b2Vec2 blackholePos=m_obs_blackhole->GetPosition();
    
    if(b2Distance(nowPos, blackholePos)<6 && b2Distance(nowPos, blackholePos)>1){
        
        b2Vec2 forceDir=blackholePos-nowPos;
        
        float r=forceDir.Normalize();
        
        forceDir*=100/(r);
        
        m_ball->ApplyForce(forceDir-Gravity, nowPos, true);
        
        
    }
    
    
}

void Game::checkBuoyancy(){
    
    
//    std::set<fixturePair>::iterator it = my_listner.m_fixturePairs.begin();
//      std::set<fixturePair>::iterator end = my_listner.m_fixturePairs.end();
//      while (it != end) {
//
//          //fixtureA is the fluid
//          b2Fixture* fixtureA = it->first;
//          b2Fixture* fixtureB = it->second;
//
//          float density = fixtureA->GetDensity();
//
//          std::vector<b2Vec2> intersectionPoints;
//          if ( my_listner.findIntersectionOfFixtures(fixtureA, fixtureB, intersectionPoints) ) {
//
//              //find centroid
//              float area = 0;
//              b2Vec2 centroid = my_listner.ComputeCentroid( intersectionPoints, area);
//
//              //apply buoyancy stuff here...
//          }
//
//          ++it;
//      }
    
    
}

void Game::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color,b2Vec2& bodyPos,float rot) {
    
    
    glPushMatrix();
    
    glTranslatef(bodyPos.x, bodyPos.y, 0.0f);
    glRotatef(rot, 0.0f, 0.0f, 1.0f);
    glBegin(GL_LINE_LOOP);
    glColor4f(color.r, color.g, color.b, 1.0f);
    for (int i = 0; i < vertexCount; i++) {
        b2Vec2 v = vertices[i];
        
        glVertex2f(v.x , v.y );
    }
    glEnd();
    glPopMatrix();
}

void Game::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color,b2Vec2& bodyPos,float rot) {
    
    
    
    glPushMatrix();
    
    glTranslatef(bodyPos.x, bodyPos.y, 0.0f);
    glRotatef(rot, 0.0f, 0.0f, 1.0f);
  
    
    
    glBegin(GL_TRIANGLE_FAN);

    glColor4f(color.r, color.g, color.b, 0.8f);
    for (int i = 0; i < vertexCount; i++) {
        b2Vec2 v = vertices[i];
        
        glVertex2f(v.x , v.y );
    }
    glEnd();
    glPopMatrix();
}

void Game::DrawCircle(const b2Vec2& center, float radius, const b2Color& color) {
    const float k_segments = 16.0f;
    const int vertexCount = 16;
    const float k_increment = 2.0f * b2_pi / k_segments;
    float theta = 0.0f;
    
    glPushMatrix();

    
    glBegin(GL_LINE_LOOP);
 
    glColor4f(color.r, color.g, color.b, 0.5f);
    GLfloat glVertices[vertexCount * 2];
    for (int32 i = 0; i < k_segments; ++i) {
        b2Vec2 v = center + radius * b2Vec2(cos(theta), sin(theta));
        glVertex2f(v.x , v.y );
        theta += k_increment;
    }
    glEnd();
    glPopMatrix();
}

void Game::DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) {
    const float k_segments = 16.0f;
    const int vertexCount = 16;
    const float k_increment = 2.0f * b2_pi / k_segments;
    float theta = 0.0f;
    
    glBegin(GL_TRIANGLE_FAN);
    
    glColor4f(color.r, color.g, color.b, 0.5f);
    
    for (int32 i = 0; i < k_segments; ++i) {
        b2Vec2 v = center + radius * b2Vec2(cos(theta), sin(theta));
        glVertex2f(v.x , v.y);
        theta += k_increment;
    }
    glEnd();
    
    
    
    
    DrawSegment(center, center + radius*axis, color);
    
}

void Game::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
    
    
    
    glColor4f(color.r, color.g, color.b, 1);
    
    glBegin(GL_LINES);

    glVertex2f(p1.x , p1.y );
    glVertex2f(p2.x , p2.y );
    glEnd();
    
}
