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
#include <list>

// #include <SDL2/SDL_image.h>

int main()
{
    chdir(SDL_GetBasePath());

    const int FPS = 60;                // set FPS
    const int frameDelay = 1000 / FPS; // delay according to FPS
    Uint32 frameStart;                 // keeps track of time (?)
    int frameTime;
    int window_width = 1200;
    int window_height = 800;
    UI ui{ window_width, window_height };
    PendulumDynamics pendulumDynamics;

    double pi = 3.141592653589793238462643383279502884197;
    double x0, y0, x1, y1, x2, y2; // link positions
    double x2_prev, y2_prev; // previous endeffector position
    std::vector<std::array<double, 2>> trajectory;
    x0 = 0; y0 = 0; // base position
    double l1 = 150, l2 = 150; // pendulum length on screen

    Pendulum pendulum(0, 0); // pendulum with initial angles
    Uint32 mouseState;
    int x, y;
    double x_conv;
    double y_conv;

    double qd1 = 0; // desired angle 1
    double qd2 = 0; // desired angle 2
    // not working when robot base at 0, 0?
    double xd = 0; // desired x (when doing inverse kinematics) (!! in meters not pixels !!)
    double yd = -1; // desired y

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
            switch (event.type) {
            case SDL_QUIT:
                quit = true;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                case SDLK_LEFT:
                    break;
                case SDLK_RIGHT:
                    break;
                case SDLK_UP:
                    break;
                case SDLK_DOWN:
                    break;
                case SDLK_ESCAPE:
                    quit = true;
                    break;
                case SDLK_c:
                    pendulumDynamics.toggleController();
                    break;
                case SDLK_r:
                    reset = true;
                    break;
                case SDLK_SPACE:
                    //pause = true;
                    break;
                default:
                    break;
                }

            case SDL_KEYUP:
                // printf("Key release detected\n");
                break;

            default:
                break;
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
                // ALSO HAVE TO RESET TURN COUNT!!!
            }

            // mouse position
            mouseState = SDL_GetMouseState(&x, &y);
            // coordinates to window center
            x = x - window_width / 2;
            y = y - window_height / 2;
            // converting pixels to endeffector position in meters
            x_conv = x * 1 / (l1 + l2); // total length of robot arm over total length in pixels
            y_conv = -y * 1 / (l1 + l2);

            // checking if desired position is inside workspace
            if (sqrt(x_conv * x_conv + y_conv * y_conv) < 1)
            {
                xd = x_conv;
                yd = y_conv; // flipping y for IK assumptions
            }
            // when mouse is outside of workspace
            else if (sqrt(x_conv * x_conv + y_conv * y_conv) >= 1)
            {
                // calculate position on unit circle (total robot length here = 1!!) -> multiply by total length
                double th1 = atan(y_conv / x_conv);
                xd = 0.99999 * cos(th1); // can't handle exactly 1
                yd = 0.99999 * sin(th1);
                if (x_conv < 0)
                {
                    xd = -xd; yd = -yd; // flipping for quadrant 2 and 3
                }
            }

            // integration
            for (int i = 0; i < 10; ++i) // two integration steps per frame
            {
                pendulumDynamics.setReceivedInputs(pendulumDynamics.inverseKinematics({ xd, yd }));
                // seperate controller? (maybe seperate IK?)
                pendulumDynamics.setReceivedStates(pendulum.getStates());
                pendulumDynamics.rungeKutta();
                pendulum.setStates(pendulumDynamics.getUpdatedStates());
            }

            // calculate coordinates of links (relative angles)
            x1 = x0 + l1 * sin(pendulum.getStates().at(0));
            y1 = y0 + l1 * cos(pendulum.getStates().at(0));
            x2 = x1 + l2 * sin(pendulum.getStates().at(0) + pendulum.getStates().at(1));
            y2 = y1 + l2 * cos(pendulum.getStates().at(0) + pendulum.getStates().at(1));
        }

        trajectory.push_back({ x2, y2 }); // store endeffector position in trajectory

        // rendering screen
        ui.clear(); // clears screen

        int width = 10; // half width of the link
        // angles of the short side of the link to the origin (makes it a bit easier to calculate)
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

        // draw trajectory (intensity dependend on recency)
        // for (int i = 1; i < trajectory.size(); i++)
        // {
        //     ui.setDrawColor(255, 255, 255, i);
        //     ui.drawLine(trajectory.at(i - 1).at(0), trajectory.at(i - 1).at(1), trajectory.at(i).at(0), trajectory.at(i).at(1));
        // }

        // only draw last X positions
        if (trajectory.size() > 200)
        {
            trajectory.erase(trajectory.begin());
        }

        ui.present(); // shows rendered objects

        frameCount += 1; // count Frame

        // frame time to limit FPS
        frameTime = SDL_GetTicks() - frameStart;
        if (frameDelay > frameTime)
        {
            SDL_Delay(frameDelay - frameTime);
        }
    }
    return 0;
}