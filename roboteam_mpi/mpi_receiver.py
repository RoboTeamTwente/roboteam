from mpi4py import MPI

def main():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()

    if rank == 1:
        while True:
            message = comm.recv(source=0, tag=11)
            print(f"Receiver: {message}")

if __name__ == "__main__":
    main()
    