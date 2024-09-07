#include "rangermocklaser.h"

RangerMockLaser::RangerMockLaser() {
}

RangerMockLaser::RangerMockLaser(unsigned int fov, unsigned int ares, int offset, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  angularResolution_ = ares;
  offset_ = offset;
  fieldOfView_ = fov;
  mockData_ = mockData;

  Laser::setAngularResolution(ares);
  Laser::setOffset(offset);
  Laser::setFieldOfView(fov);
}

unsigned int RangerMockLaser::getAngularResolution(void) {
  return angularResolution_;
}
int RangerMockLaser::getOffset(void) {
  return offset_;
}

unsigned int RangerMockLaser::getFieldOfView(void) {
  return fieldOfView_;
}

std::vector<double> RangerMockLaser::generateData() {
  return mockData_;
}

bool RangerMockLaser::setAngularResolution(unsigned int ares) {
  angularResolution_ = ares;
  return true;
}

bool RangerMockLaser::setOffset(int offset) {
  offset_ = offset;
  return true;
}

bool RangerMockLaser::setFieldOfView(unsigned int fov) {
  fieldOfView_ = fov;
  return true;
}

SensingMethod RangerMockLaser::getSensingMethod(void) {
  return SensingMethod::POINT;
}
