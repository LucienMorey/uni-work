#include "gtest/gtest.h"
#include <iostream>
#include "complex.h"

// The 'TEST' keyword signals to the gtest framework that this is a unit test
TEST(ComplexNumTest, Divide)
{
  // Two different methods of initialising a struct
  TComplex a = { .re = 2, .im = 3 };
  TComplex b = { 1, 1 };
  TComplex expected = { 2.5, 0.5 };
  TComplex answer = Complex::divide(a, b);

  // Check that the expected is equal to the actual answer
  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST(ComplexNumTest, Addition)
{
  TComplex a = { 2, 2 };
  TComplex b = { 2, 2 };
  TComplex expected = { 4, 4 };
  TComplex answer = Complex::add(a, b);

  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST(ComplexNumTest, Subtraction)
{
  TComplex a = { 2, 2 };
  TComplex b = { 2, 2 };
  TComplex expected = { 0, 0 };
  TComplex answer = Complex::subtract(a, b);

  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST(ComplexNumTest, Multiplication)
{
  TComplex a = { 2, 2 };
  TComplex b = { 2, 2 };
  TComplex expected = { 0, 8 };
  TComplex answer = Complex::multiply(a, b);

  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST(ComplexNumTest, Magnitude)
{
  TComplex a = { 3, 4 };
  double expected = 5.0;
  double answer = Complex::magnitude(a);

  EXPECT_EQ(expected, answer);
}

TEST(ComplexNumTest, Conjugate)
{
  TComplex a = { 2, 2 };
  TComplex expected = { 2, -2 };
  TComplex answer = Complex::conjugate(a);

  EXPECT_EQ(expected.re, answer.re);
  EXPECT_EQ(expected.im, answer.im);
}

TEST(ComplexNumTest, Format)
{
  TComplex a = { 2, 2 };
  std::string expected = "2 + 2i";
  std::string answer = Complex::format(a);

  EXPECT_EQ(expected, answer);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
