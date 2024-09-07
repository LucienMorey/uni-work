#include "person.h"
#include <iostream>  // Includes std::cout and friends so we can output to console

Person::Person()
{
  name_ = "None";
  age_ = 0;
}

void Person::setName(std::string name)
{
  name_ = name;
}

bool Person::setAge(int age)
{
  if (age < 0 || age > 120)
  {
    age = 0;
    return false;
  }
  else
  {
    age_ = age;
    return true;
  }
}

std::string Person::getName(void)
{
  return name_;
}

int Person::getAge(void)
{
  return age_;
}
