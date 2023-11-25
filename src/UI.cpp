#include <SDL2/SDL.h>
#include <vector>

#include "../include/UI.h"

using namespace std::chrono_literals;

UI::UI(int sizeX, int sizeY) : sizeX(sizeX), sizeY(sizeY) {
  initialize(sizeX, sizeY);
}

UI::~UI() {
  if (renderer)
    SDL_DestroyRenderer(renderer);
  if (window)
    SDL_DestroyWindow(window);
  SDL_Quit();
}

void UI::clear() {
  setDrawColor(0, 70, 110, 255);
  SDL_RenderClear(renderer);
  setDrawColor(255, 255, 255, 255);
}

void UI::present() {
  // SDL_Delay(10);
  SDL_RenderPresent(renderer);
}
// function to draw pixel on screen (not used)
void UI::drawPixel(int x, int y) {
  SDL_RenderDrawPoint(renderer, x + sizeX / 2, y + sizeY / 2);
}
// function to draw line between two points
void UI::drawLine(int _x2, int _y2, int x2, int y2) {
  SDL_RenderDrawLine(renderer, _x2 + sizeX / 2, _y2 + sizeY / 2, x2 + sizeX / 2,
                     y2 + sizeY / 2);
}

// drawing a rectangle along a line (for links)
void UI::drawTiltedRectangle(double _x1, double _y1, double _x2, double _y2,
                             double _ang, int _width) {
  this->drawLine(_x1 + _width * sin(_ang), _y1 - _width * cos(_ang),
                 _x2 + _width * sin(_ang), _y2 - _width * cos(_ang));
  this->drawLine(_x1 - _width * sin(_ang), _y1 + _width * cos(_ang),
                 _x2 - _width * sin(_ang), _y2 + _width * cos(_ang));

  this->drawLine(_x1 + _width * sin(_ang), _y1 - _width * cos(_ang),
                 _x1 - _width * sin(_ang), _y1 + _width * cos(_ang));
  this->drawLine(_x2 + _width * sin(_ang), _y2 - _width * cos(_ang),
                 _x2 - _width * sin(_ang), _y2 + _width * cos(_ang));
}

// draw trajectory (intensity dependend on recency)
void UI::drawTrajectory(std::vector<std::array<double, 2>> &_trajectory,
                        int _length) {
  for (int i = 1; i < _trajectory.size(); i++) {
    this->setDrawColor(255, 255, 255, i);
    this->drawLine(_trajectory[i - 1][0], _trajectory[i - 1][1],
                   _trajectory[i][0], _trajectory[i][1]);
  }
  // only draw last X positions
  if (_trajectory.size() > _length) {
    _trajectory.erase(_trajectory.begin());
  }
}

void UI::setDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
}

SDL_Renderer *&UI::getRenderer() // pointer reference to the renderer
{
  return renderer;
}
SDL_Window *UI::getWindow() // pointer to the window
{
  return window;
}

// initializing the UI
void UI::initialize(int sizeX, int sizeY) {
  SDL_Init(SDL_INIT_EVERYTHING);

  // Create a Window
  window = SDL_CreateWindow("Robot Arm", 0, 0, sizeX, sizeY, SDL_WINDOW_SHOWN);

  renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
  // scale window
  SDL_SetWindowSize(window, sizeX, sizeY);
  // adjust render scale
  SDL_RenderSetScale(renderer, 1, 1);
  // place window in middle of screen after scaling
  SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
}
