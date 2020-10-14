//
//  geometry.h
//  pinball_mini
//
//  Created by 이재현 on 2020/10/08.
//

#ifndef geometry_h
#define geometry_h


#include <box2d/box2d.h>
#include <iostream>
#include <vector>
#include <set>

typedef std::pair<b2Fixture*, b2Fixture*> fixturePair;


class Myb2ContactListener: public b2ContactListener{
    
public:
    std::set<fixturePair> m_fixturePairs;
    
    void BeginContact(b2Contact* contact);
  
    
    void EndContact(b2Contact* contact);

    
    float IntersectArea(b2Fixture* rect, b2Fixture* circle,float L,float H);
    
    
};




#endif /* geometry_h */
