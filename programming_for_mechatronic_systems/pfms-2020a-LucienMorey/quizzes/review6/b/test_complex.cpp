#include "gtest/gtest.h"
#include <iostream>
#include "complex.h"

//The 'TEST' keyword signals to the gtest framework that this is a unit test
TEST (ComplexNumTest, Magnitude) {
  //Two different methods of initialising a struct
  TComplex a = { .re = -2.0, .im = 3.0};
  //double expected = { 2.0, 3.0 };
  //double answer = Complex::magnitude(a);

  //Check that the expected is equal to the actual answer
  EXPECT_NEAR(3.60555127, Complex::magnitude(a), 0.001);
}


TEST (ComplexNumTest, Conjugate) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex expected = { 2.0, -3.0 };
  TComplex answer = Complex::conjugate(a);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Add) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex b = { 1, 1 };
  TComplex expected = { 3.0, 4.0 };
  TComplex answer = Complex::add(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Subtract) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex b = { 1, 1 };
  TComplex expected = { 1.0, 2.0 };
  TComplex answer = Complex::subtract(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Multiply) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex b = { 1, 1 };
  TComplex expected = { -1.0, 5.0 };
  TComplex answer = Complex::multiply(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Divide) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex b = { 1, 1 };
  TComplex expected = { 2.5, 0.5 };
  TComplex answer = Complex::divide(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
