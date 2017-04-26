#include "Optimization.h" //temp

namespace rtt {

template<typename Num, size_t N>
GradientDescent<Num, N>::GradientDescent(ScoreFunction scorer, Data data,
		Data initialSteps, std::array<std::pair<Num, Num>, N> limits)
		: scorer(scorer), data(data), steps(initialSteps), limits(limits), iterationCount(0) {}

template<typename Num, size_t N>
GradientDescent<Num, N>::GradientDescent(ScoreFunction scorer, Data data, Num initialStep, Num minimum, Num maximum)
	: scorer(scorer), data(data), iterationCount(0) {
	for (size_t i = 0; i < N; i++) {
		limits[i] = { minimum, maximum };
		steps[i] = initialStep;
	}
}

template<typename Num, size_t N>
void GradientDescent<Num, N>::singleIteration() {

	Num currentSum = Num(0), newSum = Num(0);
	for (Num n : data) currentSum += n;

	for (size_t i = 0; i < N; i++) {
		data[i] += steps[i];
		newSum += data[i];
	}

	double newScore = scorer(data);
	if (lastScore > newScore) {
		for (size_t i = 0; i < N; i++) steps[i] /= Num(-2);
	}
	lastScore = newScore;
	lastDivergence = newSum - currentSum;
	iterationCount++;
}

template<typename Num, size_t N>
bool GradientDescent<Num, N>::iterateToLimit(size_t limit) {
	if (iterationCount >= limit) return false;
	while (iterationCount < limit) singleIteration();
	return true;
}

template<typename Num, size_t N>
bool GradientDescent<Num, N>::iterateUntilScoreAtLeast(Num threshold, size_t limit) {
	if (lastScore >= threshold) return false;
	size_t count = 0;
	while (lastScore < threshold && count < limit) {
		singleIteration();
		count++;
	}
	return count < limit;
}

template<typename Num, size_t N>
bool GradientDescent<Num, N>::iterateUntilConvergent(Num threshold, size_t limit) {
	if (lastDivergence <= threshold) return false;
	size_t count = 0;
	while (lastDivergence > threshold && count < limit) {
		singleIteration();
		count++;
	}
	return count < limit;
}



template<typename Num, size_t N>
Num GradientDescent<Num, N>::getLastScore() const {
	return lastScore;
}

template<typename Num, size_t N>
Num GradientDescent<Num, N>::getLastDivergence() const {
	return lastDivergence;
}


template<typename Num, size_t N>
size_t GradientDescent<Num, N>::getIterationCount() const {
	return iterationCount;
}

}
