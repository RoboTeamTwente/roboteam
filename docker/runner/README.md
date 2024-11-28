
In a ray or distributed computing cluster, the terms "head node" and "worker nodes" refer to different roles that containers play in the cluster. The head node is the master node in a Ray cluster. You typically have one head node. Worker nodes are the containers that execute the jobs, in parallel. You can have as many worker nodes as you want.

-----------------------------------------------------------

## Installing Kuberay

curl <https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3> | bash
helm repo add kuberay <https://ray-project.github.io/kuberay-helm/>
helm repo update
helm install kuberay-operator kuberay/kuberay-operator --namespace ray-system --create-namespace

<https://docs.ray.io/en/latest/cluster/kubernetes/user-guides/config.html>
The above source was used for creating the ray-cluster.yaml

Installing kubernetes and minikubernetes, you can follow this guide to check out how to install and run them: <https://medium.com/@areesmoon/installing-minikube-on-ubuntu-20-04-lts-focal-fossa-b10fad9d0511>

Use 'pip install ray' and then 'pip show ray' to get your version of ray.

-----------------------------------------------------------

After you have both kubernetes and ray, use the following command to create a cluster: kubectl apply -f ray-cluster.yaml
This cluster launches a ray head node and one worker node. Launch the external simulator using kubectl apply -f simulator.yaml

'kubectl get pods'-> this is will give you the cluster name

Use to forward the needed port to the ray service: kubectl port-forward svc/<cluster name> 8265:8265
This is the port that will be used inside ray_jobs.py, where we submit the jobs to ray.

-----------------------------------------------------------

## Useful commands

kubectl apply -f ray-cluster.yaml
kubectl delete -f ray-cluster.yaml
helm install kuberay-operator ray/kuberay-operator
helm uninstall kuberay-operator
kubectl port-forward svc/roboteam-ray-cluster-head-nodeport 8265:8265 6379:6379 10001:10001 8000:8000 &
minikube start -p ray --nodes 2 --memory 4000 --cpus 3
