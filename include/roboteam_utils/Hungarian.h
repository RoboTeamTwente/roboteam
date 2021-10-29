///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class Hungarian.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <iostream>
#include <vector>

#include "Vector2.h"

using namespace std;

namespace rtt {

class Hungarian {
   public:
    Hungarian() = default;
    static double Solve(vector<vector<double>>& DistMatrix, vector<int>& Assignment);
    static std::vector<std::pair<Vector2, Vector2>> getOptimalPairs(std::vector<Vector2> set1, std::vector<Vector2> set2);
    static std::unordered_map<int, Vector2> getOptimalPairsIdentified(const std::unordered_map<int, Vector2>& positions, std::vector<Vector2> targetLocations);

   private:
    static bool validateInput(std::vector<Vector2> const& set1, std::vector<Vector2> const& set2);
    static void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nOfRows, int nOfColumns);
    static void buildassignmentvector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns);
    static void computeassignmentcost(int* assignment, double* cost, double* distMatrix, int nOfRows);
    static void step2a(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                       int nOfColumns, int minDim);
    static void step2b(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                       int nOfColumns, int minDim);
    static void step3(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                      int nOfColumns, int minDim);
    static void step4(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                      int nOfColumns, int minDim, int row, int col);
    static void step5(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, bool* coveredColumns, bool* coveredRows, int nOfRows,
                      int nOfColumns, int minDim);
};

}  // namespace rtt
#endif