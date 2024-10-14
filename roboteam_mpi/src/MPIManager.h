#ifndef MPI_MANAGER_H
#define MPI_MANAGER_H

#include <mpi.h>
#include <vector>
#include <string>

class MPIManager {
public:
    static void init(int& argc, char**& argv);
    static void finalize();
    static int getRank();
    static int getSize();
    
    static void send(const void* data, int count, MPI_Datatype datatype, int dest, int tag);
    static void recv(void* data, int count, MPI_Datatype datatype, int source, int tag);
    static void bcast(void* data, int count, MPI_Datatype datatype, int root);

private:
    static bool initialized;
    static int rank;
    static int size;

    MPIManager() = delete;  // Prevent instantiation
};