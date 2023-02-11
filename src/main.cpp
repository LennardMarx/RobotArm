#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <ratio>

#include "../include/UI.h"
#include "../include/pendulum.h"
#include "../include/pendulum_dynamics.h"


#include <utility>
#include <string>
#include <vector>
#include <unistd.h>

// #include <SDL2/SDL_image.h>

int main()
{
    chdir(SDL_GetBasePath());

    const int FPS = 60;                // set FPS
    const int frameDelay = 1000 / FPS; // delay according to FPS
    Uint32 frameStart;                 // keeps track of time (?)
    int frameTime;

    UI ui{ 1200, 800 };
    PendulumDynamics pendulumDynamics;

    double pi = 3.141592653589793238462643383279502884197;
    double x0, y0, x1, y1, x2, y2; // link positions
    x0 = 0; y0 = 0; // base position
    double l1 = 200, l2 = 100; // pendulum length on screen

    Pendulum pendulum(0, 0); // pendulum with initial angles

    double qd1 = 0;
    double qd2 = 0;

    bool reset = false;
    bool pause = false;
    bool quit = false;
    bool controllerOff = false;

    int frameCount = 0;

    SDL_Event event;

    while (!quit)
    {
        frameStart = SDL_GetTicks(); // limit framerate (see end of while loop)

        while (SDL_PollEvent(&event) != 0)
        {
            // stop when pressing "x" (?)
            if (event.type == SDL_QUIT)
            {
                quit = true;
            }
        }
        // pause the game
        if (pause)
        {
            // do nothing
        }
        else
        {
            // reset
            if (reset == true)
            {
                pendulum.setStates({ 0, 0, 0, 0 });
                reset = false;
            }

            // integration
            for (int i = 0; i < 10; ++i) // two integration steps per frame
            {
                pendulumDynamics.setReceivedInputs({ qd1, qd2 });

                // controller here?

                pendulumDynamics.setReceivedStates(pendulum.getStates());
                pendulumDynamics.rungeKutta();
                pendulum.setStates(pendulumDynamics.getUpdatedStates());
            }

            // calculate coordinates of links (absolute angles)
            // x1 = x0 + l1 * sin(pendulum.getStates().at(0));
            // y1 = y0 + l1 * cos(pendulum.getStates().at(0));
            // x2 = x1 + l2 * sin(pendulum.getStates().at(1));
            // y2 = y1 + l2 * cos(pendulum.getStates().at(1));

            // calculate coordinates of links (relative angles)
            x1 = x0 + l1 * sin(pendulum.getStates().at(0));
            y1 = y0 + l1 * cos(pendulum.getStates().at(0));
            x2 = x1 + l2 * sin(pendulum.getStates().at(0) + pendulum.getStates().at(1));
            y2 = y1 + l2 * cos(pendulum.getStates().at(0) + pendulum.getStates().at(1));
        }

        // rendering screen
        ui.clear(); // clears screen

        int width = 10; // half width
        double ang1 = pi / 2 - pendulum.getStates().at(0);
        double ang2 = pi / 2 - (pendulum.getStates().at(0) + pendulum.getStates().at(1));

        // draw as rectangle (make function!)
        // link 1
        ui.drawLine(x0 + width * sin(ang1), y0 - width * cos(ang1), x1 + width * sin(ang1), y1 - width * cos(ang1));
        ui.drawLine(x0 - width * sin(ang1), y0 + width * cos(ang1), x1 - width * sin(ang1), y1 + width * cos(ang1));

        ui.drawLine(x0 + width * sin(ang1), y0 - width * cos(ang1), x0 - width * sin(ang1), y0 + width * cos(ang1));
        ui.drawLine(x1 + width * sin(ang1), y1 - width * cos(ang1), x1 - width * sin(ang1), y1 + width * cos(ang1));



        // link 2
        ui.drawLine(x1 + width * sin(ang2), y1 - width * cos(ang2), x2 + width * sin(ang2), y2 - width * cos(ang2));
        ui.drawLine(x1 - width * sin(ang2), y1 + width * cos(ang2), x2 - width * sin(ang2), y2 + width * cos(ang2));

        ui.drawLine(x1 + width * sin(ang2), y1 - width * cos(ang2), x1 - width * sin(ang2), y1 + width * cos(ang2));
        ui.drawLine(x2 + width * sin(ang2), y2 - width * cos(ang2), x2 - width * sin(ang2), y2 + width * cos(ang2));


        // draw pendulum links (middle line)
        // ui.drawLine(x0, y0, x1, y1);
        // ui.drawLine(x1, y1, x2, y2);

        ui.present(); // shows rendered objects

        frameCount += 1; // count Frame

        // control sequence
        if (frameCount >= 100 && frameCount <= 300)
        {
            qd1 = -pi / 6;
            qd2 = 2 * pi / 3;
        }
        if (frameCount >= 300)
        {
            if (qd1 <= pi / 6)
            {
                qd1 += 0.01;
                qd2 -= 0.01;
            }
        }
        if (frameCount >= 600)
        {
            qd1 = pi - 0.001;
            qd2 = 0;
        }
        if (frameCount >= 800)
        {
            qd1 = 0;
            qd2 = 0;
        }
        if (frameCount >= 1000)
        {
            frameCount = 0;
        }

        // frame time to limit FPS
        frameTime = SDL_GetTicks() - frameStart;
        if (frameDelay > frameTime)
        {
            SDL_Delay(frameDelay - frameTime);
        }
    }
    return 0;
}