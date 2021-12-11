#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <iostream>
#include <vector>
#include <bits/stdc++.h>

namespace rtt {
    using namespace std;
    using Matrix = vector<vector<double>>;

    class Hungarian {
    public:
        Hungarian() = default;
        ~Hungarian() = default;

        vector<int> Solve(const Matrix& cost);

    private:
        void initVariables(const Matrix& cost);

        void printCost();

        void printMask();

        void printAssignment();

        void rowSubtraction();

        void starZeros();

        void coverStars();

        void primeUncoveredZeros();

        void findZero(int& row, int& col);

        bool starRow(int row);

        void findStarCol(int row, int& col);

        void starPrimes();

        void findStarRow(int col, int& row);

        void findPrimeCol(int row, int& col);

        void augmentPath();

        void clearCovers();

        void clearPrimes();

        void findMinCost();

        void returnAssignments();

        int step{ 1 };
        bool optimal{ false };

        int numRobots{};
        int numRoles{};
        Matrix m_cost;

        vector<bool> RowCover;
        vector<bool> ColCover;
        Matrix mask;

        int path_row_0{};
        int path_col_0{};
        int pathCount{};
        vector<vector<int>> path; // SHOULD NOT BE SQUARE

        vector<int> m_assignment;
    };
} // namespace rtt
#endif