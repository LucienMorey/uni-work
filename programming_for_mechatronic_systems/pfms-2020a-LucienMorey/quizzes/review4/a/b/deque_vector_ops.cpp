#include <iostream>
#include <deque>
#include <chrono>
#include <random>
#include <vector>
#include <algorithm>

//! Sample function for the creating elements of the deque
void populateDeque(std::deque<double>& values, int num_values, double mean, double std_dev) {

	// Create a random number generator and seed it from
	// the system clock so numbers are different each time

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution (mean,std_dev);

	for (int i = 0; i < num_values; i++)
	{
		values.push_back(distribution(generator));
	}
}

//! Sample function for printing elements of the deque
void printDeque(std::deque<double>& values) {
	// Loop through the deque and print the values
	for (auto value : values) {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}

void populateVector(std::vector<double>& values, int num_values, double mean, double std_dev) {

	// Create a random number generator and seed it from
	// the system clock so numbers are different each time

    //http://www.cplusplus.com/reference/random/normal_distribution/normal_distirbution/

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  	std::default_random_engine generator (seed);
  	std::normal_distribution<double> distribution (mean,std_dev);

	for (int i = 0; i < num_values; i++)
	{
		values.push_back(distribution(generator));
	}
}

void printVector(std::vector<double>& values) {
	// Loop through the deque and print the values
	for (auto value : values) {
		std::cout << value << " ";
	}
	std::cout << std::endl;
}  
  
void bubbleSortDeque(std::deque<double> &values){

	bool swapped = false;
	int num_values = values.size();

	while (!swapped)
	{
		swapped = false;
		for (int j = 1; j < num_values; j++){

			for (int i = 1; i < num_values; i++)
			{
				if (values.at(i-1) > values.at(i))
				{
					std::swap(values.at(i-1), values.at(i));
					swapped = true;
				}
			}
		}
	}
}

void bubbleSortVector(std::vector<double> &values){

	bool swapped = false;
	int num_values = values.size();

	while (!swapped)
	{
		swapped = false;
		for (int j = 1; j < num_values; j++){

			for (int i = 1; i < num_values; i++)
			{
				if (values.at(i-1) > values.at(i))
				{
					std::swap(values.at(i-1), values.at(i));
					swapped = true;
				}
			}
		}
	}
}

int main() {

    ////////////////////////////////////////////////

	// Create an empty deque
	std::deque<double> values;

	// Populate it with random numbers
    int num_values;
	int mean = 8;
	int std_dev = 4;
    std::cout << "How many elements do you wish to generate : ";
    
	// Ensure value is between 0 and ARRAY_MAX_SIZE
    bool inRange = false;
    while (!inRange) {
        while(!(std::cin >> num_values)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input, Try again: ";
        }
        if (num_values <= 0) {
            std::cout << "Value must be over" << 0 << ", Try again: ";
        } else {
            inRange = true;
        }
    }

	populateDeque(values, num_values, mean, std_dev);

	// Print the contents of the deque
	std::cout << "Deque values:";
	printDeque(values);

    ////////////////////////////////////////////////

	// Create an empty vector
	std::vector<double> vec_values;

	// Populate it with random numbers
	populateVector(vec_values, num_values, mean, std_dev);

	// Print the contents of the vector
	std::cout << "Vector values:";
	printVector(vec_values);

    ////////////////////////////////////////////////

    // Bubble sort one of your containers
	//bubbleSortDeque(values);
	bubbleSortVector(vec_values);

	// Print the contents of the container
	//std::cout << "Sorted deque values:";
	//printDeque(values);

	std::cout << "Sorted vector values:";
	printVector(vec_values);

	return 0;
} 
