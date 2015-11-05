#include <iostream>
#include <memory>

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
    void OnEvent(SDL_Event&& evt);
    void Render();
    
    
private:
    bool m_isRunning;
    double m_time;
    SDL_Window *m_window;
    std::shared_ptr<PhysicsEngine> m_physicsEngine;
    std::shared_ptr<Vessel> m_vessel;
};


Game::Game()
    : m_isRunning(false)
    , m_time(0.)
    , m_physicsEngine(std::shared_ptr<PhysicsEngine>(new PhysicsEngine()))
{
    
    m_window = SDL_CreateWindow(
        "Navigation simulator",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
        640, 480, 
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN);
    

    // m_vessel = std::make_shared<Vessel>();
    m_vessel = std::shared_ptr<Vessel>(new Vessel());

    m_physicsEngine->add_body(m_vessel);
    
}

void Game::OnEvent(SDL_Event&& evt)
{
    if(evt.type==SDL_QUIT)
        m_isRunning = false;    
}

void Game::Render()
{
	Vector3d position = m_vessel->getGeneralizedPosition();
	Vector3d velocity = m_vessel->getGeneralizedVelocity();

    int w, h;
    SDL_GetWindowSize(m_window, &w, &h);

    float ratio = (float)w / (float)h;

    // Create an OpenGL context associated with the window.
    SDL_GLContext glcontext = SDL_GL_CreateContext(m_window);


    // now you can make GL calls.
    glClearColor(0.f,0.f,.0f,1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    
    glLoadIdentity();

    glScalef(.003f, .003f * ratio, 1.f);

    glTranslated(position(1), position(0), 0.);
    glRotated(- position(2) * 180. / M_PI, 0, 0, 1);


    glBegin( GL_TRIANGLES );
    glVertex2d( - m_vessel->getBeam() / 2, - m_vessel->getLength() / 2 );
    glVertex2d( m_vessel->getBeam() / 2, - m_vessel->getLength() / 2 );
    glVertex2d( 0.0f, m_vessel->getLength() / 2 );
    glEnd();

    glColor3f(1., 0., 0.);
    glBegin( GL_LINES );
    glVertex2f( 0.f, 0.f );
    glVertex2d( velocity(1), velocity(0));
    glEnd();


    SDL_GL_SwapWindow(m_window);

    // Once finished with OpenGL functions, the SDL_GLContext can be deleted.
    SDL_GL_DeleteContext(glcontext); 
}

void Game::run() 
{
    m_isRunning = true;
    double time_delta = 0.001;
    
    while(m_isRunning)
    {
        // Handling event
        SDL_Event evt;
        while(SDL_PollEvent(&evt))
        {
            this->OnEvent(std::move(evt));
        }
        
        double old_time = m_time;
        // Physic simulation
        for(int i = 0; i<100; i++)
        {
        	m_physicsEngine->timestep(time_delta);
        	m_time += time_delta;
        }

        SDL_Delay((m_time - old_time) * 100.);
        // m_physicsEngine.print_all_positions();
        
        // Rendering
        this->Render();
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
