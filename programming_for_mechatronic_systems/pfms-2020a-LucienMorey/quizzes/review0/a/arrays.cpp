#include <iostream> // Includes std::cout and friends so we can output to console
#include <random>   // Includes the random number generator
#include <chrono>   // Includes the system clock
#include <limits>
#include <cmath>


// Create a macro (#define) to represent the max possible array size
#define ARRAY_MAX_SIZE 100

// function to print elements of the array
void printArray(double* x, int xSize){
    for (double *ip = x; ip<(x+xSize); *ip++){
        std::cout << "*ip = " << *ip << std::endl;
    }
}

//// function to print elements of the array
//void printArray(double x[], int xSize){
//  for (int i = 0; i<xSize; i++) {
//      std::cout << "x[" << i << "] = " << x[i] << std::endl;
//  }
//}

// function to populate array with random numbers
void populateWithRandomNumbers(double num_array[], int& array_size, int num_elements) {

    //we select a seed for the random generator, so it is truly random (neve the same seed)
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    std::uniform_real_distribution<> value_distribution(0,10.0);

    int oldArraySize = array_size - num_elements;

    // generate the required amount of random numbers
    for (int i=oldArraySize; i<oldArraySize+num_elements; i++) {
        num_array[i] = value_distribution(generator);
    }
}

void threshold(double array[],int arraySize)
{
    double sum = 0, mean = 0, var = 0, stddev = 0, thres = 0;
    for(int i=0;i<arraySize;i++)
    {
        sum += array[i];
    }

    mean = sum/arraySize;

    for(int i=0;i<arraySize;i++)
    {
        var += pow(array[i]-mean,2);
    }
    var /= arraySize;

    stddev = sqrt(var);

    thres = stddev + mean;

    for(int i=0;i<arraySize;i++)
    {
        if(array[i]>thres)
        {
            std::cout << array[i] << std::endl;
        }
    }
}

void makeVector(double array[],int arraySize)
{
    std::vector<double> vec(array,array+arraySize);

    std::cout << std::endl << "Vector: " << std::endl;

    for(int i=0;i<arraySize;i++)
    {
        std::cout << vec.at(i) << std::endl;
    }

}

// Every executable needs a main function which returns an int
int main () {

    // Create an array x of doubles with 10 elements
    int arraySize=10;
    // Populate the elements of array on creating of array, each element [i] has value i (INITIALISER LIST)
    double x[arraySize] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

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
    int arraySize2 = arraySize + num;

    double y[arraySize2];

    for(int i=0;i<arraySize;i++)
    {
        y[i] = x[i];
    }

    // Populate array with random numbers
    populateWithRandomNumbers(y,arraySize2,num);

    // Print array
    printArray(y,arraySize2);

    threshold(y,arraySize2);

    makeVector(y,arraySize2);

    // Main function should return an integer
    return 0;
}
