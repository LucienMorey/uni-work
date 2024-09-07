#include "analysis.h"
#include "analysisinterface.h"
#include <iostream>
#include "math.h"

Analysis::Analysis()
{

}

//! TODO - TASK 3: Implement the missing function(s) in Analysis Class
//!
//! HINT: Analysis inherits from AnalysisInterface which is an abstract class
//! What type of class is AnalysisInterface, what does this mean?

std::vector<bool> Analysis::intersectsLine(){
    std::vector<bool> bool_vec;
    double D=line_.getValD();
    double dr=line_.getValDr();
    //std::cout<<D<<std::endl;
    //std::cout<<dr<<std::endl;
    double discriminant;
        for(int i=0; i<shapes_.size();i++){
            double radius=sqrt(shapes_.at(i)->getArea()/M_PI);
            discriminant=(pow(radius,2)*pow(dr,2))-pow(D,2);
                if(discriminant>=0)
                    {bool_vec.push_back(true);}
                else
                    {bool_vec.push_back(false);}
        }

    return bool_vec;
}

//! Use the following webiste to assist in developing the code
//! https://mathworld.wolfram.com/Circle-LineIntersection.html
//!
//! BONUS QUESTION (FOR DISCUSSION)
//! At the moment we are implementing check intersect for a Circle
//! as the only question used.
//! If we had a Rectangle shape, how could we differentiate between the approach
//! to intersect for the Rectangle and the Circle?
