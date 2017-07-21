#pragma once

#include "Vector2.h"
#include <functional>
#include <type_traits>

namespace rtt {
    
/*
 * Finds the value in a given range which produces the maximum result in a given function.
 * A greater initial step with higher refinement lowers the risk of finding a local maximum, but 
 * it's also slower.
 */
double optimizeLinear(
    double from,                                // The low end of the range
    double to,                                  // The high end of the range
    double step,                                // The initial step size
    int refinement,                             // The amount of times to refine (=half) the step size before returning the best result
    std::function<double(double)> scoreFunc     // The scoring function. Higher values are better.
);

Vector2 optimizeVector(
    const Vector2& center,
    double minXDev,
    double maxXDev,
    double minYDev,
    double maxYDev,
    double step,
    int refinement,
    std::function<double(const Vector2&)> scoreFunc
);

Vector2 sampleForVector(
    const Vector2& center,
    double minXDev,
    double maxXDev,
    double minYDev,
    double maxYDev,
    int samples,
    std::function<double(const Vector2&)> scoreFunc
);

/**
 * \class GradientDescent
 * \brief Optimizes a vector of numbers against a given function.
 * \tparam Num Any numeric type. If it's not numeric, it won't compile.
 * \tparam N The dimensionality of the problem.
 *
 * The score function should return greater values for better results.
 */
template<typename Num, size_t N>
class GradientDescent {
public:
	static_assert(std::is_arithmetic<Num>{}, "GradientDescent Num must be an arithmetic type");
	typedef std::array<Num, N> Data;
	typedef std::function<Num(const Data&)> ScoreFunction;

	/**
	 * \brief Constructs a GradientDescent by explicitly providing each parameter.
	 */
	GradientDescent(ScoreFunction scorer, Data initialData, Data initialSteps,
			std::array<std::pair<Num, Num>, N> limits);

	/**
	 * \brief Constructs a GradientDescent where each parameter has the same initial step size, and
	 * the same upper and lower limits.
	 */
	GradientDescent(ScoreFunction scorer, Data initialData, Num initialStep, Num minimum, Num maximum);

	/**
	 * \brief Perform a single iteration of the algorithm.
	 */
	void singleIteration();

	/**
	 * \brief Continuously run the algorithm until getIterationCount() >= limit.
	 * \return true if the algorithm ran at least once, false if not.
	 */
	bool iterateToLimit(size_t limit);

	/**
	 * \brief Continously run the algorithm until either the score exceeds
	 * the given threshold, or until it has run limit times (irrespective of the total iteration count).
	 */
	bool iterateUntilScoreAtLeast(Num threshold, size_t limit = 1000);

	/**
	 * \brief Continously run the algorithm until the total change in the data was at most
	 * the given threshold, or until it has run limit times (irrespective of the total iteration count).
	 */
	bool iterateUntilConvergent(Num threshold, size_t limit = 1000);

	Num getLastScore() const;
	Num getLastDivergence() const;
	size_t getIterationCount() const;
	Data getCurrentValues() const;

private:
	ScoreFunction scorer;
	std::array<std::pair<Num, Num>, N> limits;
	Data data;
	Data steps;
	Num lastDivergence;
	Num lastScore;
	bool justFlipped;
	size_t iterationCount;
};

}

#include "Optimization.tpp"
