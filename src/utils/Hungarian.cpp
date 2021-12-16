#include "Hungarian.h"

namespace rtt {

vector<int> Hungarian::Solve(const Matrix& cost) {
    initVariables(cost);
    while (!optimal) {
        switch (step) {
            case 1:
                rowSubtraction();
                break;
            case 2:
                starZeros();
                break;
            case 3:
                coverStars();
                break;
            case 4:
                primeUncoveredZeros();
                break;
            case 5:
                starPrimes();
                break;
            case 6:
                findMinCost();
                break;
            case 7:
                optimal = true;
                returnAssignments();
                break;
            default:
                break;
        }
    }
    return m_assignment;
}

void Hungarian::initVariables(const Matrix& cost) {
    numRoles = static_cast<int>(cost.size());      // number of columns
    numRobots = static_cast<int>(cost[0].size());  // number of rows

    ColCover.resize(numRoles);
    RowCover.resize(numRobots);

    path.resize(numRobots * 2, vector<int>(numRobots * 2));
    mask.resize(numRoles, vector<double>(numRobots));
    m_cost.resize(numRoles, vector<double>(numRobots));
    m_cost = cost;

    m_assignment.resize(numRoles);
}

void Hungarian::printCost() {
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            std::cout << m_cost[role][robot] << " ";
        }
        std::cout << std::endl;
    }
}

void Hungarian::printMask() {
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            std::cout << mask[role][robot] << " ";
        }
        std::cout << std::endl;
    }
}

void Hungarian::printAssignment() {
    for (auto i{0}; i < numRoles; ++i) {
        cout << m_assignment[i] << " ";
    }
    cout << endl;
}

void Hungarian::rowSubtraction() {
    double min_in_row;
    for (auto robot{0}; robot < numRobots; ++robot) {
        // find the lowest cost in a row
        min_in_row = m_cost[0][robot];
        for (auto role{1}; role < numRoles; ++role) {
            if (m_cost[role][robot] < min_in_row) min_in_row = m_cost[role][robot];
        }
        // then subtract it from all other cost in the same row
        for (auto role{0}; role < numRoles; ++role) {
            m_cost[role][robot] -= min_in_row;
        }
    }
    step = 2;
}

void Hungarian::starZeros() {
    // search for zero costs and mark them in the mask matrix if the row and column has not been covered yet
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            if (m_cost[role][robot] == 0 && !RowCover[robot] && !ColCover[role]) {
                mask[role][robot] = 1;  // star
                RowCover[robot] = true;
                ColCover[role] = true;
            }
        }
    }
    clearCovers();
    step = 3;
}

void Hungarian::coverStars() {
    // check if all columns are assigned to a row in the mask matrix (i.e. if every role has a robot assigned)
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            if (mask[role][robot] == 1) ColCover[role] = true;
        }
    }
    int role_count{0};
    for (int role{0}; role < numRoles; ++role) {
        if (ColCover[role]) ++role_count;
    }
    if (role_count == numRoles)
        step = 7;  // successfully optimized assignment
    else
        step = 4;
}

void Hungarian::primeUncoveredZeros() {
    int robot{-1};
    int role{-1};
    bool primed{false};

    while (!primed) {
        findZero(robot, role);
        if (robot == -1) {  // if no zeros were found there is nothing left to prime
            primed = true;
            step = 6;
        } else {
            mask[role][robot] = 2;  // mark as primed in mask matrix
            if (starRow(robot)) {   // if there is a stared zero in the same row as the primed zero
                // change the stared zero's cover to overlap with the primed zero's cover
                findStarCol(robot, role);
                RowCover[robot] = true;
                ColCover[role] = false;
            } else {
                // save the smallest uncovered value and move on
                path_row_0 = robot;
                path_col_0 = role;
                primed = true;
                step = 5;
            }
        }
    }
}

void Hungarian::findZero(int& row, int& col) {
    bool found{false};
    // reset input in case this is not the first while(!primed) iteration
    row = -1;
    col = -1;
    // return the coordinates of the first uncovered zero found in the cost matrix
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            if (m_cost[role][robot] == 0 && !RowCover[robot] && !ColCover[role]) {
                row = robot;
                col = role;
                found = true;
                break;
            }
        }
        if (found) break;
    }
}

bool Hungarian::starRow(int row) {
    // returns whether the mask matrix has a star in a given row
    bool found{false};
    for (int role{0}; role < numRoles; ++role) {
        if (mask[role][row] == 1) found = true;
    }
    return found;
}

void Hungarian::findStarCol(int row, int& col) {
    // returns the column of a stared zero in a given row
    col = -1;
    for (int role{0}; role < numRoles; ++role) {
        if (mask[role][row] == 1) col = role;
    }
}

void Hungarian::starPrimes() {
    bool done{false};
    int robot{-1};
    int role{-1};

    pathCount = 1;
    path[pathCount - 1][0] = path_row_0;
    path[pathCount - 1][1] = path_col_0;

    while (!done) {
        findStarRow(path[pathCount - 1][1], robot);
        if (robot > -1) {
            ++pathCount;
            path[pathCount - 1][0] = robot;
            path[pathCount - 1][1] = path[pathCount - 2][1];
        } else
            done = true;
        if (!done) {
            findPrimeCol(path[pathCount - 1][0], role);
            ++pathCount;
            path[pathCount - 1][0] = path[pathCount - 2][0];
            path[pathCount - 1][1] = role;
        }
    }
    augmentPath();
    clearCovers();
    clearPrimes();
    step = 3;
}

void Hungarian::findStarRow(int col, int& row) {
    // returns the row of a stared zero in a given column
    row = -1;
    for (int robot{0}; robot < numRobots; ++robot) {
        if (mask[col][robot] == 1) row = robot;
    }
}

void Hungarian::findPrimeCol(int row, int& col) {
    // returns the column of a primed zero in a given row
    for (int role{0}; role < numRoles; ++role) {
        if (mask[role][row] == 2) col = role;
    }
}

void Hungarian::augmentPath() {
    for (int p{0}; p < pathCount; ++p) {
        if (mask[path[p][1]][path[p][0]] == 1)
            mask[path[p][1]][path[p][0]] = 0;
        else
            mask[path[p][1]][path[p][0]] = 1;
    }
}

void Hungarian::clearCovers() {
    // reset all covered rows and columns
    for (int robot{0}; robot < numRobots; ++robot) {
        RowCover[robot] = false;
    }
    for (int role{0}; role < numRoles; ++role) {
        ColCover[role] = false;
    }
}

void Hungarian::clearPrimes() {
    // reset all primes stored in the mask matrix
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            if (mask[role][robot] == 2) mask[role][robot] = 0;
        }
    }
}

void Hungarian::findMinCost() {
    // find the lowest uncovered cost
    double minCost{INT_MAX};
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            if (!RowCover[robot] && !ColCover[role] && minCost > m_cost[role][robot]) minCost = m_cost[role][robot];
        }
    }
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            // add the lowest cost to all covered rows
            if (RowCover[robot]) m_cost[role][robot] += minCost;
            // subtract the lowest cost from all uncovered columns
            if (!ColCover[role]) m_cost[role][robot] -= minCost;
        }
    }
    step = 4;
}

void Hungarian::returnAssignments() {
    for (auto role{0}; role < numRoles; ++role) {
        for (auto robot{0}; robot < numRobots; ++robot) {
            if (mask[role][robot] == 1) m_assignment[role] = robot;
        }
    }
}
}  // namespace rtt