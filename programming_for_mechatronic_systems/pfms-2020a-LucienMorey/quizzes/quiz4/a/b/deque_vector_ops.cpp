#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>

double mean = 8.0;
double std_dev = 4.0;

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev)
{
  // Create a random number generator and seed it from
  // the system clock so numbers are different each time

  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(mean, std_dev);

  for (auto i = 1; i <= num_values; i++)
  {
    values.push_back(distribution(generator));
  }

  // http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/
}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double>& values)
{
  // Loop through the deque and print the values
  for (auto value : values)
  {
    std::cout << value << " ";
  }
  std::cout << std::endl;
}

void printVector(std::vector<double>& values)
{
  // Loop through the deque and print the values
  for (auto value : values)
  {
    std::cout << value << " ";
  }
  std::cout << std::endl;
}

void populateVector(std::vector<double>& values, int num_values, double mean, double std_dev)
{
  unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(mean, std_dev);

  for (auto i = 1; i <= num_values; i++)
  {
    values.push_back(distribution(generator));
  }
}

void bubbleSortVector(std::vector<double>& values)
{
  bool swapped = true;
  double temp;
  while (swapped)
  {
    swapped = false;
    for (auto value_it = values.begin(); value_it < (values.end() - 1); ++value_it)
    {
      // std::cout << *(value_it+1)<< std::endl;
      if (*(value_it) > (*(value_it + 1)))
      {
        temp = *value_it;
        *(value_it) = *(value_it + 1);
        *(value_it + 1) = temp;
        swapped = true;
      }
    }
  }
}

int main()
{
  ////////////////////////////////////////////////

  // Create an empty deque
  std::deque<double> values;
  // Populate it with random numbers
  int num_of_values;
  std::cout << "please enter the number of elements you wish to generate: " << std::endl;
  while (!(std::cin >> num_of_values))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }

  populateDeque(values, num_of_values, mean, std_dev);

  // Print the contents of the deque

  printDeque(values);

  ////////////////////////////////////////////////

  // Create an empty vector
  std::vector<double> vector_values;

  std::cout << "please enter the number of elements you wish to generate: " << std::endl;
  while (!(std::cin >> num_of_values))
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Invalid input. Please enter an integer value and try again: " << std::endl;
  }

  // Populate it with random numbers
  populateVector(vector_values, num_of_values, mean, std_dev);

  // Print the contents of the vector
  printVector(vector_values);

  ////////////////////////////////////////////////

  // Bubble sort one of your containers
  bubbleSortVector(vector_values);

  // Print the contents of the container
  printVector(vector_values);

  return 0;
}
