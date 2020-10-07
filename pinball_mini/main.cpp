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
//
//    glfwWindowHint(GL_SAMPLES, 4);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
////    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
////    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    
    WINDOW_WIDTH=800;
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
    
    
    
    glClearColor( 0.2, 0.2, 0.2, 0.5f);
    
//    glEnable(GL_DEPTH_TEST);
//    // Accept fragment if it closer to the camera than the former one
//    glDepthFunc(GL_LESS);
//    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    return 0;
}




void setCamera(){
    
   
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, float(400)/float(400), 0.1, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 50, 0, 0, 0, 0, 1, 0);

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
