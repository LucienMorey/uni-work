#include <iostream>  // Includes std::cout and friends so we can output to console
#include <vector>
#include "person.h"

void crowdGreeting(std::vector<Person> crowd);

int main(void)
{
  // Create the person 'alice'
  Person alice;
  alice.setName("Alice");
  alice.setAge(32);

  // Create the person 'bob' and print his age
  Person bob;
  bob.setName("Bob");

  if (bob.setAge(-62))
  {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  }
  else
  {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  // Create the person 'carol'
  Person carol;
  carol.setName("Carol");
  carol.setAge(72);

  //! TODO Create a 'crowd' using a vector container of people
  std::vector<Person> crowd;
  crowd.push_back(alice);
  crowd.push_back(bob);
  crowd.push_back(carol);
  //! TODO Create a function that greets the oldest crowd member
  //!
  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;
  crowdGreeting(crowd);

  return 0;
}

/**
 * @brief Function to greet the oldest member in a vector of people
 *
 * @param crowd std::vector of Person so be checked
 */

void crowdGreeting(std::vector<Person> crowd)
{
  Person oldestPerson;

  for (auto person : crowd)
  {
    if (person.getAge() > oldestPerson.getAge())
    {
      oldestPerson = person;
    }
  }

  std::cout << "Hello " << oldestPerson.getName() << ", you're our most distinguished member" << std::endl;
}
