#ifndef ANALYSISINTERFACE_H
#define ANALYSISINTERFACE_H

#include <vector>
#include "shape.h"
#include "line.h"

// The AnalysisInterface is a class which specifies the minimum
// required interface for your Analisys class must inherit from it
//
// Note that AnalysisInterface and Analysis only know about shape
// and have no knowledge of the Derived Classes
class AnalysisInterface
{
public:
    AnalysisInterface(){}; // Default contrsuctor is empty

    // Accepts container of Shapes
    virtual void setShapes(std::vector<Shape*> shapes) = 0;

    // Accepts a line 
    virtual void setLine(Line line) = 0;

    // Returns a container of bools if intersects exist
    virtual std::vector<bool> intersectsLine() = 0;

};

#endif // ANALYSISINTERFACE_H
