#include <iostream> // Includes std::cout and friends so we can output to console
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>


// Create a macro (#define) to represent the max possible array size
#define ARRAY_MAX_SIZE 100

// function to print elements of the array
void printPointerArray(double* x, int xSize){
  double *test = x;
  std::cout<< test << " " << x << std::endl;
  for (double *ip = x; ip<(x+xSize); *ip++){
      std::cout << "*ip = " << *ip << std::endl;
  }
  std::cout<<std::endl;
}

// function to print elements of the array
void printArray(double x[], int xSize){
  for (int i = 0; i<xSize; i++) {
      std::cout << "x[" << i << "] = " << x[i] << std::endl;
  }
  std::cout<<std::endl;
}

// function to populate array with random numbers
void populateWithRandomNumbers(double num_array[], int& array_size, int num_elements) {

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0,10.0);
    // generate the required amount of random numbers
    for (int i=array_size; i<array_size+num_elements; i++) {
        num_array[i] = value_distribution(generator);
    }
    //let's update the array size
    array_size+=num_elements;
}

void printOutlier(double x[], int arraySize) {
    double total = 0.0;
    //get the sum of elements in the array until a blank element is found
    for( int i = 0; i<arraySize; i++){
        total = total + x[i];
    }

    //calculate and print mean
    double mean = total/double(arraySize);
    std::cout << "The mean is " << mean << std::endl;

    //reset elements and compute standard deviation
    total = 0;
    for(int i = 0; i < arraySize; i++){
        total = total + pow((x[i]-mean),2);
    }

    double stdDeviation = sqrt(total/arraySize);
    std::cout << "The std Deviation is " << stdDeviation << std::endl;

    double comparator = mean + stdDeviation;
     std::cout<< "the elements greater than the mean and std deviation are" << std::endl;
    for(int i = 0; i < arraySize; i++){
        if(x[i] > comparator){
            std::cout<< x[i]<< std::endl;
        }
    }
    std::cout<<std::endl;
}

void vectorAssigner(double x[], int arraySize, std::vector<double> &y){
    for(int i = 0; i < arraySize; i++){
        y.push_back(x[i]);
    }

    std::cout<< "the value of the vector are" << std::endl; 
    for(auto i : y){
        std::cout<< i << std::endl;
    }
}

// Every executable needs a main function which returns an int
int main () {

    std::vector<double> data;
    
    // Create an array x of doubles with 10 elements
    int arraySize=10;
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[ARRAY_MAX_SIZE] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // Print array
    printArray(x,arraySize);

    // Ask user to specify how many additional numbers are to be generated
    int num;
    std::cout << "How many random elements do you wish to generate : ";
     while(!(std::cin >> num)){
         std::cin.clear();
         std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
         std::cout << "Invalid input, Try again: ";
     }

    // Populate array with random numbers
    populateWithRandomNumbers(x,arraySize,num);

    // Print array
    printPointerArray(x,arraySize);

    printOutlier(x,arraySize);

    vectorAssigner(x,arraySize,data);

    // Main function should return an integer
    return 0;
}
