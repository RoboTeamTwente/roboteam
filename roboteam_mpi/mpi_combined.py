# mpi_combined.py
from mpi4py import MPI
import sys

def main():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()

    print(f"Process {rank}: I am rank {rank} out of {size} processes")
    sys.stdout.flush()

    if rank == 0:
        number = 42
        print(f"Process {rank}: Sending number {number} to rank 1")
        sys.stdout.flush()
        comm.send(number, dest=1, tag=11)
        print(f"Process {rank}: Number {number} sent to rank 1")
        sys.stdout.flush()
    elif rank == 1:
        print(f"Process {rank}: Waiting to receive number from rank 0")
        sys.stdout.flush()
        number = comm.recv(source=0, tag=11)
        print(f"Process {rank}: Received number {number} from rank 0")
        sys.stdout.flush()

if __name__ == "__main__":
    main()