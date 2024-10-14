from mpi4py import MPI

def main():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()

    if rank == 0:
        message = "Test"
        comm.send(message, dest=1, tag=11)
        print(f"Sender: {message}")
if __name__ == "__main__":
    main()