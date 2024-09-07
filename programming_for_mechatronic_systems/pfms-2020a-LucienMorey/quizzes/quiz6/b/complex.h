#ifndef COMPLEX_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define COMPLEX_H

#include <string>

struct TComplex {
  double re;
  double im;
};  // A new struct type defining a complex number

class Complex {
public:
  //Find the magnitude of a complex number
  static double magnitude(TComplex);
  //Find the complex conjugate of a complex number
  static TComplex conjugate(TComplex);
  //Add two complex numbers a & b
  static TComplex add(TComplex, TComplex);
  //Subtract two complex numbers a & b
  static TComplex subtract(TComplex, TComplex);
  //Multiply two complex numbers a & b
  static TComplex multiply(TComplex, TComplex);
  //Divide two complex numbers a & b
  static TComplex divide(TComplex, TComplex);
  //Format a TComplex type to a string
  static std::string format(TComplex);
};

#endif
