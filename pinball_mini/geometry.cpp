////
////  geometry.cpp
////  pinball_mini
////
////  Created by 이재현 on 2020/10/08.
////
//
//
//#include <box2d/box2d.h>
//#include <iostream>
//#include <vector>
//#include <set>
//#include <math.h>
//
//#include "geometry.h"
//
////class member variable
//
//
//
//
//void Myb2ContactListener::BeginContact(b2Contact *contact){
//    b2Fixture* fixtureA = contact->GetFixtureA();
//    b2Fixture* fixtureB = contact->GetFixtureB();
//
//    if ( fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody )
//        m_fixturePairs.insert( std::make_pair(fixtureA, fixtureB) );
//    else if ( fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody )
//        m_fixturePairs.insert( std::make_pair(fixtureB, fixtureA) );
//}
//
//void Myb2ContactListener:: EndContact(b2Contact* contact)
//{
//    b2Fixture* fixtureA = contact->GetFixtureA();
//    b2Fixture* fixtureB = contact->GetFixtureB();
//
//    if ( fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody )
//        m_fixturePairs.erase( std::make_pair(fixtureA, fixtureB) );
//    else if ( fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody )
//        m_fixturePairs.erase( std::make_pair(fixtureB, fixtureA) );
//}
//
//float Myb2ContactListener::IntersectArea(b2Fixture* rect, b2Fixture* circle,float L,float H){
//
//    if(rect->GetShape()->GetType()!=b2Shape::e_polygon || circle->GetShape()->GetType()!=b2Shape::e_circle){
//        return -0.1f;
//    }
//
//    b2Vec2 rect_center=rect->GetBody()->GetPosition();
//    b2Vec2 circle_center=circle->GetBody()->GetPosition();
//    float radius=circle->GetShape()->m_radius;
//
//    if(circle_center.y-radius>H/2+rect_center.y){//no intersect
//        return 0.0f;
//    }else if(circle_center.y+radius<H/2+rect_center.y ){
//        return radius*radius*b2_pi;
//    }else{
//        float x=(rect_center.y+H/2)+radius-circle_center.y;
//        float ret=0.0f;
//        ret=radius*radius*acos(1-(x/radius))-sqrt(2*x*radius-x*x)*(radius-x);
//        return ret;
//    }
//
//
//    return 0.0f;
//}
//
//
//
//
//
//
//
//
