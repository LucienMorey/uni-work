#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "analysisinterface.h"

class Analysis : public AnalysisInterface
{
public:
  Analysis();

  void setShapes(std::vector<Shape*> shapes) {shapes_=shapes;};

  void setLine(Line line) {line_=line;};

  //Returns a container of bools that indicates if the correspoing shape intersects the line
  std::vector<bool> intersectsLine();


private:
  std::vector<Shape*> shapes_;
  Line line_;

};

#endif // ANALYSIS_H
