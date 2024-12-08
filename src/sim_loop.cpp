#include "../include/sim_loop.h"

// SimLoop::SimLoop() : pendulum(Pendulum(0, 0)) {} // member initializer list
SimLoop::SimLoop() {}
SimLoop::~SimLoop() {}

void SimLoop::run() {
#ifdef __EMSCRIPTEN__
  emscripten_set_main_loop_arg(mainloop, &ctx, -1, 1);
#else
  while (!ctx.helperVars.getQuit())
    mainloop(&ctx);
#endif
}

// void run() {
void SimLoop::mainloop(void *arg) {
  context *ctx = static_cast<context *>(arg);

  ctx->frameStart = SDL_GetTicks(); // limit framerate (see end of while loop)

  // check for Keyboard inputs;
  ctx->eventChecks.checkEvents(ctx->helperVars, ctx->pendulumDynamics);

  // pause the game
  if (ctx->pause) {
    // do nothing
  } else {
    // reset
    if (ctx->helperVars.getReset() == true) {
      ctx->pendulum.setStates({0, 0, 0, 0});
      ctx->helperVars.toggleReset();
      // reset = false;
      //  ALSO HAVE TO RESET TURN COUNT!!!
    }

    // mouse position
    ctx->mouseState = SDL_GetMouseState(&ctx->x, &ctx->y);
    // coordinates to window center
    ctx->x = ctx->x - ctx->window_width / 2;
    ctx->y = ctx->y - ctx->window_height / 2;
    // converting pixels to endeffector position in meters
    ctx->x_conv =
        ctx->x * 1 /
        (ctx->l1 +
         ctx->l2); // total length of robot arm over total length in pixels
    ctx->y_conv = -ctx->y * 1 / (ctx->l1 + ctx->l2);

    // checking if desired position is inside workspace
    if (sqrt(ctx->x_conv * ctx->x_conv + ctx->y_conv * ctx->y_conv) < 1) {
      ctx->xd = ctx->x_conv;
      ctx->yd = ctx->y_conv; // flipping y for IK assumptions
    }
    // when mouse is outside of workspace
    else if (sqrt(ctx->x_conv * ctx->x_conv + ctx->y_conv * ctx->y_conv) >= 1) {
      // calculate position on unit circle (total robot length here = 1!!)
      // -> multiply by total length
      double th1 = atan(ctx->y_conv / ctx->x_conv);
      ctx->xd = 0.99999 * cos(th1); // can't handle exactly 1
      ctx->yd = 0.99999 * sin(th1);
      if (ctx->x_conv < 0) {
        ctx->xd = -ctx->xd;
        ctx->yd = -ctx->yd; // flipping for quadrant 2 and 3
      }
    }

    // integration// two integration steps per frame
    for (int i = 0; i < 10; ++i) {
      ctx->pendulumDynamics.setReceivedInputs(
          ctx->pendulumDynamics.inverseKinematics({ctx->xd, ctx->yd}));
      // seperate controller? (maybe seperate IK?)
      ctx->pendulumDynamics.setReceivedStates(ctx->pendulum.getStates());
      ctx->pendulumDynamics.rungeKutta();
      ctx->pendulum.setStates(ctx->pendulumDynamics.getUpdatedStates());
    }

    // calculate coordinates of links (relative angles)
    ctx->x1 = ctx->x0 + ctx->l1 * sin(ctx->pendulum.getStates().at(0));
    ctx->y1 = ctx->y0 + ctx->l1 * cos(ctx->pendulum.getStates().at(0));
    ctx->x2 = ctx->x1 + ctx->l2 * sin(ctx->pendulum.getStates().at(0) +
                                      ctx->pendulum.getStates().at(1));
    ctx->y2 = ctx->y1 + ctx->l2 * cos(ctx->pendulum.getStates().at(0) +
                                      ctx->pendulum.getStates().at(1));
  }

  ctx->helperVars.getTrajectory().push_back(
      {ctx->x2, ctx->y2}); // store endeffector position in trajectory

  // rendering screen
  ctx->ui.clear(); // clears screen

  int width = 10; // half width of the link
  // angles of the short side of the link to the origin (makes it a bit
  // easier to calculate)
  double ang1 = ctx->pi / 2 - ctx->pendulum.getStates().at(0);
  double ang2 = ctx->pi / 2 - (ctx->pendulum.getStates().at(0) +
                               ctx->pendulum.getStates().at(1));

  // drawing the links
  ctx->ui.drawTiltedRectangle(ctx->x0, ctx->y0, ctx->x1, ctx->y1, ang1, width);
  ctx->ui.drawTiltedRectangle(ctx->x1, ctx->y1, ctx->x2, ctx->y2, ang2, width);

  // draw trajectory (intensity dependend on recency), define trajectory
  // length
  if (ctx->helperVars.getTrajOn()) {
    ctx->ui.drawTrajectory(ctx->helperVars.getTrajectory(), 200);
  }
  ctx->ui.present(); // shows rendered objects

  ctx->frameCount += 1; // count Frame

  // frame time to limit FPS
  ctx->frameTime = SDL_GetTicks() - ctx->frameStart;
  if (ctx->frameDelay > ctx->frameTime) {
    SDL_Delay(ctx->frameDelay - ctx->frameTime);
  }
}
