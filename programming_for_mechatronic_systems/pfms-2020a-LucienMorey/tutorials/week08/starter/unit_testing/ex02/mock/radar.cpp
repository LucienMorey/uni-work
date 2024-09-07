#include "radar.h"

Radar::Radar(){
  fieldOfView_ = 60;
  angularResolution_ = 20;
}

void Radar::setAngularResolution(unsigned int ares) {
  angularResolution_ = ares;
}
void Radar::setOffset(unsigned int degrees) {
  offset_ = degrees;
}
