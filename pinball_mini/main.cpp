//
//  main.cpp
//  glut_ex
//
//  Created by 이재현 on 2020/10/05.
//



#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "game.hpp"
#include "signal.h"



GLFWwindow* window;
Game* m_game;
int WINDOW_WIDTH=0;
int WINDOW_HEIGHT=0;


int init(){
    if (!glfwInit()) {
        std::cout << "glfw init failed!!\n";
        return 1;
    }
    
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    
    WINDOW_WIDTH=1000;
    WINDOW_HEIGHT=1000;
    
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "opengl_playground", NULL, NULL);
    
    if (!window) {
        std::cout << "window creation failed!\n";
        glfwTerminate();
        return 1;
    }
    
    glfwMakeContextCurrent(window);
    
    
    glewExperimental = GL_TRUE;
    
    GLenum errorCode = glewInit();
    if (GLEW_OK != errorCode) {
        
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    
    
    
    glClearColor( 0.25, 0.25, 0.25, 1.0f);
    
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
  
    
    return 0;
}




void setCamera(){
    
    
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, float(400)/float(400), 0.1, 1000.0);
  
    gluLookAt(0, 5, 70, 0, 5, 0, 0, 1, 0);
    
    glFlush();
    
    
    
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_A && action == GLFW_PRESS){
        m_game->doFlip(GameSignal::DO_FLIP);
    }else{
        m_game->doFlip(GameSignal::DO_NOT_FLIP);
    }
    
    if(key==GLFW_KEY_R ){
        m_game=new Game();
        m_game->createMap();
    }
    
}


int main(int argc, char ** argv) {
    // insert code here...
    
    init();
    
    m_game=new Game();
    
    m_game->createMap();
    
    glfwSetKeyCallback(window, key_callback);
        
   
    while (!glfwWindowShouldClose(window)) {
        setCamera();
        
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        m_game->step();
        m_game->render();
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        
    }
    
    
    
    return 0;
}
