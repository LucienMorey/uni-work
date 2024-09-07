#include "ranger.h"

Ranger::Ranger() {
}

Ranger::Ranger(unsigned int fov, unsigned int ares, int offset, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  angularResolution_ = ares;
  offset_ = offset;
  fieldOfView_ = fov;
  mockData_ = mockData;
}

unsigned int Ranger::getAngularResolution(void) {
  return angularResolution_;
}
int Ranger::getOffset(void) {
  return offset_;
}

unsigned int Ranger::getFieldOfView(void) {
  return fieldOfView_;
}

std::vector<double> Ranger::generateData() {
  return mockData_;
}

void Ranger::setAngularResolution(unsigned int ares) {
  angularResolution_ = ares;
}

void Ranger::setOffset(int offset) {
  offset_ = offset;
}

void Ranger::setFieldOfView(unsigned int fov) {
  fieldOfView_ = fov;
}
