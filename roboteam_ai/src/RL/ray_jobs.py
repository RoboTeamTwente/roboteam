import ray
import requests

#connect to the head node, to submit jobs
#ray.init(address=os.getenv('RAY_ADDRESS'))

#@ray.remote
#def run_simulation(simulation_name, port):
#    print(f"Running simulation {simulation_name} on port {port}")
#    return f"Simulation {simulation_name} completed on port {port}"

#result = ray.get(compute_square.remote(4))
#print(result)

#result
#iiii
job_payload = {
    "entrypoint": "python3 example_ray_job.py"
}

response = requests.post("http://localhost:8265/api/jobs/", json=job_payload)
print(response.json())
