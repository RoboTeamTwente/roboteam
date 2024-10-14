#include <mpi.h>
#include <iostream>

int main(int argc, char *argv[]){

MPI_Init(&argc, &argv);

int world_size; //World size is total amount of processes
MPI_Comm_size(MPI_COMM_WORLD, &world_size);

int world_rank;
MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

if (world_rank == 0)
{
// Sending a message
const int message = 42;
MPI_Send(&message, //
1, 
MPI_INT, 
1, 
0, 
MPI_COMM_WORLD);

std::cout << "Process 0 sends number " << message << " to process 1\n";
}
else if (world_rank == 1)
{
// Receiving a message
int received_message;
MPI_Recv(&received_message, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
std::cout << "Process 1 received number " << received_message << " from process 0\n";
}

MPI_Finalize();
return 0;

}