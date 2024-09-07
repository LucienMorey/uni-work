#include <iostream> // Includes std::cout and friends so we can output to console
#include <vector> // Including vectors.
#include "person.h"

void greetOldest(std::vector<Person> &vect) {
    std::vector<int> ages;
    for (auto x:vect) {
        ages.push_back(x.getAge()); //assigns the ages of the crowd members to a new vector 'ages'
    }

    //these if statements are used to compare which age is higher and print the member accordingly

    if (ages[0] > ages[1] && ages[0] > ages[2]) {
        std::cout << "Hello " << vect[0].getName() << ", you're our most distinguished member." << std::endl;
    }

    if (ages[1] > ages[0] && ages[1] > ages[2]) {
        std::cout << "Hello " << vect[1].getName() << ", you're our most distinguished member." << std::endl;
    }

    if (ages[2] > ages[0] && ages[2] > ages[1]) {
        std::cout << "Hello " << vect[2].getName() << ", you're our most distinguished member." << std::endl;
    }

    //std::cout << crowd[1].getName() << crowd[2].getName() << "vect[3].getName()" << std::endl;

}


int main (void) {
  //Create the person 'alice'
  Person alice("Alice", 29);


  //Create the person 'bob' and print his age
  Person bob("Bob", 25);


  if (bob.getAge() > 0 && bob.getAge() < 120) {
    std::cout << bob.getName() << "'s age is " << bob.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << bob.getName() << "'s age" << std::endl;
  }

  //Create the person 'carol'
  Person carol("Carol", 20);

  //check alice and carol's ages

  if (alice.getAge() > 0 && alice.getAge() < 120) {
    std::cout << alice.getName() << "'s age is " << alice.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << alice.getName() << "'s age" << std::endl;
  }

  if (carol.getAge() > 0 && carol.getAge() < 120) {
    std::cout << carol.getName() << "'s age is " << carol.getAge() << std::endl;
  } else {
    std::cout << "Something is wrong with " << carol.getName() << "'s age" << std::endl;
  }



  //! TODO Create a 'crowd' using a vector container of people

    std::vector<Person> crowd = {alice, bob, carol};  //Creates vector 'crowd' with 'Person' class objects.

  //! TODO Create a function that greets the oldest crowd member

    greetOldest(crowd); //see above for function definition


  //! The greeting should be: "Hello NAME, your our most distinguished member" << std::endl;

  return 0;
}
