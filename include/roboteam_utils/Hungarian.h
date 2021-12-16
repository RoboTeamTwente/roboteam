#ifndef HUNGARIAN_H
#include <bits/stdc++.h>

#include <iostream>
#include <vector>

/// The Hungarian is a linear assignment algorithm meant to solve a cost matrix problem in O(n^3)
/// The Dealer sends the cost of moving each robot to each role and receives an assignment vector where the index represents the roles index (equal to the rows column in the cost
/// matrix) This implementation of the Hungarian Assignment Algorithm (aka Munkres) is a slightly modified version of the one introduced in a paper by Robert Pilgrim
/// https://www.researchgate.net/publication/290437481_Tutorial_on_Implementation_of_Munkres'_Assignment_Algorithm
/// Max Thielen Dec 15th 2021

namespace rtt {
using namespace std;
using Matrix = vector<vector<double>>;

class Hungarian {
   public:
    Hungarian() = default;
    ~Hungarian() = default;

    /**
     * @brief A switch case with each step of the Hungarian Assignment (step updated each function call)
     * @param cost matrix
     */
    vector<int> Solve(const Matrix& cost);

   private:
    /**
     * @brief Initiate member variables
     * @param cost matrix
     */
    void initVariables(const Matrix& cost);
    /**
     * @brief Print the cost matrix
     */
    void printCost();
    /**
     * @brief Print the mask matrix
     */
    void printMask();
    /**
     * @brief Print the assignment vector (index=role_id)
     */
    void printAssignment();
    /**
     * @brief Subtract smallest value in each row
     */
    void rowSubtraction();
    /**
     * @brief Search for zero costs and mark them in the mask matrix if the row and column has not been covered yet
     */
    void starZeros();
    /**
     * @brief Check if all columns are assigned to a row in the mask matrix (i.e. if every role has a robot assigned)
     */
    void coverStars();
    /**
     * @brief Search for uncovered zero costs and mark them in the mask matrix as primed
     */
    void primeUncoveredZeros();
    /**
     * @brief Search for uncovered zero costs
     */
    void findZero(int& row, int& col);
    /**
     * @brief checks for a star in given row
     * @param row in the cost matrix (i.e. robot id)
     * @return whether the mask matrix has a star in a given row
     */
    bool starRow(int row);
    /**
     * @brief finds a column of a stared zero in a given a row
     * @param row in the cost matrix (i.e. robot id)
     * @param column in the cost matrix (i.e. role id)
     * @return reference of the column
     */
    void findStarCol(int row, int& col);
    /**
     * @brief Check for primes in the mask matrix and redistribute stars so that each row and column has one star
     */
    void starPrimes();
    /**
     * @brief finds a row of a stared zero in a given column
     * @param row in the cost matrix (i.e. robot id)
     * @param column in the cost matrix (i.e. role id)
     * @return reference of the row
     */
    void findStarRow(int col, int& row);
    /**
     * @brief finds a column of a primed zero in a given row
     * @param row in the cost matrix (i.e. robot id)
     * @param column in the cost matrix (i.e. role id)
     * @return reference of the column
     */
    void findPrimeCol(int row, int& col);
    /**
     * @brief Redistribute stars in the mask matrix
     */
    void augmentPath();
    /**
     * @brief Reset all covered rows and columns
     */
    void clearCovers();
    /**
     * @brief Reset all primes stored in the mask matrix
     */
    void clearPrimes();
    /**
     * @brief Find the lowest uncovered cost
     */
    void findMinCost();
    /**
     * @brief fill assignment vector
     */
    void returnAssignments();

    int step{1};
    bool optimal{false};

    int numRobots{};
    int numRoles{};
    Matrix m_cost;

    vector<bool> RowCover;
    vector<bool> ColCover;
    Matrix mask;

    int path_row_0{};
    int path_col_0{};
    int pathCount{};
    vector<vector<int>> path;

    vector<int> m_assignment;
};
}  // namespace rtt
#endif