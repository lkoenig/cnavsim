#include <iostream>

#include "SDL2/SDL.h"
#include "SDL2/SDL_opengl.h"

#include "vessel.hpp"
#include "physics.hpp"

class Game {
public:
    Game();
    virtual ~Game();
    
    void run();
    
private:
    void OnEvent(SDL_Event evt);
    void Render();
    
    
private:
    bool m_isRunning;
    double m_time;
    SDL_Window *m_window;
    PhysicsEngine m_physicsEngine;
    Vessel m_vessel;
};


Game::Game()
    : m_isRunning(false)
    , m_time(0.)
{
    
    m_window = SDL_CreateWindow(
        "Navigation simulator",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
        640, 480, 
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    
    m_physicsEngine.add_body(&m_vessel);
    
}

void Game::OnEvent(SDL_Event evt)
{
    if(evt.type==SDL_QUIT)
        m_isRunning = false;    
}

void Game::Render()
{
    // Create an OpenGL context associated with the window.
    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);

    // now you can make GL calls.
    glClearColor(0.f,0.f,1.f,1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glBegin( GL_QUADS ); 
    glVertex2f( -0.5f, -0.5f ); 
    glVertex2f( 0.5f, -0.5f ); 
    glVertex2f( 0.5f, 0.5f ); 
    glVertex2f( -0.5f, 0.5f ); 
    glEnd();
    
    SDL_GL_SwapWindow(m_window);

    // Once finished with OpenGL functions, the SDL_GLContext can be deleted.
    SDL_GL_DeleteContext(glcontext); 
}

void Game::run() 
{
    m_isRunning = true;
    double time_delta = 0.1;
    
    while(m_isRunning)
    {
        // Handling event
        SDL_Event evt;
        while(SDL_PollEvent(&evt))
        {
            this->OnEvent(evt);
        }
        
        // Physic simulation
        m_physicsEngine.timestep(time_delta);
        m_physicsEngine.print_all_positions();
        
        // Rendering
        this->Render();
        
        // Time increase
        m_isRunning &= m_time < 1800.;
        m_time += time_delta;
    }
    
}

Game::~Game(){
    SDL_DestroyWindow(m_window);
}

int main(int argc, char **argv) {
    // std::cout << "NavSim" << std::endl;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    Game *game = new Game();

    game->run();
    
    SDL_Quit();
    return 0;

}
