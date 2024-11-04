import ray
#Case 1: the function will be distributed across the ray cluster
@ray.remote
def compute_square(x):
    return x * x

#Case 2: the class will be distributes across th ray cluster
@ray.remote
class Counter:
    def __init__(self):
        self.count = 0

    def increment(self):
        self.count += 1
        return self.count

#Testing case 1:
future = compute_square.remote(5)
result = ray.get(future)
print(f"Result: {result}")

#submiting multiple tasks in parallel
futures = [compute_square.remote(i) for i in range(10)]
results = ray.get(futures)
print(f"Results: {results}")
#Testing case 2, using the class:
counter = Counter.remote()
future_count = counter.increment.remote()
print(f"Count: {ray.get(future_count)}")
