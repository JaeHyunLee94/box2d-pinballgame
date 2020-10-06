////
////  controller.cpp
////  sfml_rigidBody
////
////  Created by 이재현 on 2020/10/04.
////
//
//#include "controller.hpp"
//#include "signal.h"
//#include <iostream>
//
////Controller::Controller(sf::RenderWindow& window,Game& game){
////
////    m_game=&game;
////    m_window=&window;
////
////
////}
//
//void Controller:: processInput(){
//    
//    sf::Event event;
//    
//    //     handle all events
//    while ( m_window->pollEvent( event ) )
//    {
//        
//        ImGui::SFML::ProcessEvent(event);
//        switch ( event.type )
//        {
//            case sf::Event::Closed:
//                m_window->close();
//                break;
//        }
//    }
//    
//    
//    
//    if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
//    {
//        int MouseX = sf::Mouse::getPosition(*m_window).x;
//        int MouseY = sf::Mouse::getPosition(*m_window).y;
//        m_game->createBox(MouseX,MouseY);
//
//    }
//    
//    
//    //flip or not
//    if(sf::Keyboard::isKeyPressed(sf::Keyboard::A)){
//        std::cout <<"pressed\n";
//        m_game->doFlip(GameSignal::DO_FLIP);
//    }else{
//        std::cout <<"not pressed\n";
//        m_game->doFlip(GameSignal::DO_NOT_FLIP);
//    }
//    
//    
//    
//    //process imgui input
//    {
//        ImGui::SFML::Update(*m_window, deltaClock.restart());
//        // Create a window called "My First Tool", with a menu bar.
//        ImGui::Begin("My First Tool", &my_tool_active, ImGuiWindowFlags_MenuBar);
//        ImGui::SetWindowFontScale(1.6);
//        ImGui::SetWindowSize(ImVec2(400,900));
//        
//        if (ImGui::BeginMenuBar())
//        {
//            if (ImGui::BeginMenu("File"))
//            {
//                if (ImGui::MenuItem("Open..", "Ctrl+O")) { /* Do stuff */ }
//                if (ImGui::MenuItem("Save", "Ctrl+S"))   { /* Do stuff */ }
//                if (ImGui::MenuItem("Close", "Ctrl+W"))  { my_tool_active = false; }
//                ImGui::EndMenu();
//            }
//            ImGui::EndMenuBar();
//        }
//        
//        // Edit a color (stored as ~4 floats)
//        ImGui::ColorEdit4("Color", my_color);
//
//        // Plot some values
//        const float my_values[] = { 0.2f, 0.1f, 1.0f, 0.5f, 0.9f, 2.2f };
//        ImGui::PlotLines("Frame Times", my_values, IM_ARRAYSIZE(my_values));
//
//        // Display contents in a scrolling region
//        ImGui::TextColored(ImVec4(1,1,0,1), "Important Stuff");
//        ImGui::BeginChild("Scrolling");
//        for (int n = 0; n < 50; n++)
//            ImGui::Text("%04d: Some text", n);
//        ImGui::EndChild();
//        ImGui::End();
//        
//        ImGui::SFML::Render(*m_window);
//    
//    }
//    
//    
//}
//
