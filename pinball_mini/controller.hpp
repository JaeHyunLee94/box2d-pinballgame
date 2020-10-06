//
//  controller.hpp
//  sfml_rigidBody
//
//  Created by 이재현 on 2020/10/04.
//

#ifndef controller_hpp
#define controller_hpp
#include "game.hpp"





class Controller{
    
    
public:
    
    
    
    

    Game* m_game;
    
    bool my_tool_active=true;
    float my_color[4]={0,0,0,0};
    
//    Controller(sf::RenderWindow& m_window,Game& m_game);
    
    void processInput();
    
    
    
};

#endif /* controller_hpp */
