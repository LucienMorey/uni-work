#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <limits>

using std::cout;
using std::endl;
using std::vector;

// Declare the sum function ahead of main, normally we would do this in a header
// Use of & (ampersand) means we pass `numbers` by reference, avoiding a copy
// The risk of using a reference is we can change the original object
// We eliminate this risk by using the const keyword
double sum(const vector<double> &numbers);

int main () {
    //TODO Create a vector of doubles with 4 values
    int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine gen(seed);
    std::uniform_real_distribution<double> val(0,100);
    vector<double> vec;
    for(int i=0;i<4;i++){
        vec.push_back(val(gen));
    }
    //TODO Add a value to the end/back
    vec.insert(vec.begin(),val(gen));

    //TODO Modify the 3rd value
    vec[2]=val(gen);

    //TODO Print out the numbers
    // Using a Range-based for loop with the auto keyword 
    for(auto i : vec){
         cout<<i<<" ";
    }
    cout<<endl;
    //TODO Compute the sum via sun function and print sum
    double s=sum(vec);
    cout<<"The sum of above numbers is :"<<s<<endl;
    return 0;
}

// Define the sum function, the signature must match exactly
double sum(const vector<double> &numbers) {
//TODO Use an iterator
double total=0;
vector<double>::iterator it;
    for(auto it=std::begin(numbers);it!=std::end(numbers);it++){}
        total = std::accumulate(std::begin(numbers), std::end(numbers), 0.0);


    return total;
}

//Final Q3(B) - V2
