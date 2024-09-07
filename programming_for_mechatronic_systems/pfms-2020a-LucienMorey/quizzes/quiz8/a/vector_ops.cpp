#include <iostream>
#include <vector>

int main()
{
  unsigned int rows = 2;
  unsigned int cols = 5;

  std::vector<std::vector<double>> matrix;  // We will be creating a matrix of elements  2x numValues

  matrix.resize(rows);      // reserve 2 rows
  matrix[0].reserve(cols);  // reserve 1st row to have numValues of elements
  matrix[1].reserve(cols);  // reserve 2nd row to have numValues of elements

  for (unsigned int idx = 0; idx < cols; idx++)
  {
    matrix[0].push_back(idx);  // loop to push back idx to first row
    matrix[1].push_back(idx);  // loop to push back idx to second row
  }

  // Let's print each each element of each row
  for (auto row : matrix)
  {
    for (auto elem : row)
    {
      std::cout << elem << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}
