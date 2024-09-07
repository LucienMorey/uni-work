#include <iostream>
#include <vector>
#include <random>
#include <chrono>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(const vector<double>& numbers);

double generateRandomData();

int main()
{
  // TODO Create a vector of doubles with 4 values
  vector<double> vect = { generateRandomData(), generateRandomData(), generateRandomData(), generateRandomData() };
  cout << "vector after task 1" << endl;
  for (auto x : vect)
  {
    cout << x << ", ";
  }
  cout << endl;

  // TODO Add a value to the end/back
  // NOTE inserting at beginning and end because provided TODO and quiz readme seem to conflict
  vect.insert(vect.begin(), generateRandomData());
  vect.insert(vect.end(), generateRandomData());
  cout << "vector after task 2" << endl;
  for (auto x : vect)
  {
    cout << x << ", ";
  }
  cout << endl;

  // TODO Modify the 3rd value
  vect.at(2) = generateRandomData();
  cout << "vector after task 3" << endl;
  for (auto x : vect)
  {
    cout << x << ", ";
  }
  cout << endl;

  // TODO Print out the numbers
  // Using a Range-based for loop with the auto keyword
  cout << "vector after task 4" << endl;
  for (auto x : vect)
  {
    cout << x << ", ";
  }
  cout << endl;

  // TODO Compute the sum via sun function and print sum
  cout << "vector sum is " << sum(vect) << endl;

  return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double>& numbers)
{
  double total = 0.0;
  // TODO Use an iterator
  total = std::accumulate(numbers.begin(), numbers.end(), total);
  return total;
}

double generateRandomData()
{
  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> randomiser(0.0, 100);

  return randomiser(generator);
}