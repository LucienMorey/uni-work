#include "laser.h"

Laser::Laser(){
  fieldOfView_ = 180;
  angularResolution_ = 10;
}

void Laser::setAngularResolution(unsigned int ares) {
  angularResolution_ = ares;
}

void Laser::setOffset(unsigned int degrees) {
  offset_ = degrees;
}
