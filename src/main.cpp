#include <iostream>

#include "SDL.h"

#include "vessel.hpp"
#include "physics.hpp"

int main(int argc, char **argv) {
    // std::cout << "NavSim" << std::endl;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window *window;
    window = SDL_CreateWindow(
        "Navigation simulator",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
        640, 480, 
        SDL_WINDOW_SHOWN);
    PhysicsEngine *physics = new PhysicsEngine();
    Vessel *vessel = new Vessel();

    physics->add_body(vessel);
    double time_delta = .1; // 0.1 should be sufficient

    for (double t = 0.0; t < 1800.; t += time_delta) {
        physics->timestep(time_delta);
        physics->print_all_positions();
    }

    delete vessel;
    delete physics;
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;

}
