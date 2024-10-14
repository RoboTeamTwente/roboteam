#include "MPIManager.h"
#include <stdexcept>

bool MPIManager::initialized = false;
int MPIManager::world_rank;
int MPIManager::world_size;

void MPIManager::init(int& argc, char**& argv) {
    if (!initialized) {
        MPI_Init(&argc, &argv);
        MPI_Comm_rank(MPI_COMM_WORLD, &world_rank); // Unique ID for every process
        MPI_Comm_size(MPI_COMM_WORLD, &world_size); // Total number of processes
        initialized = true;
    }
}

void MPIManager::finalize() {
    if (initialized) {
        MPI_Finalize();
        initialized = false;
    }
}

int MPIManager::getRank() {
    if (!initialized) throw std::runtime_error("MPI not initialized");
    return rank;
}

int MPIManager::getSize() {
    if (!initialized) throw std::runtime_error("MPI not initialized");
    return size;
}

void MPIManager::send(const void* data, int count, MPI_Datatype datatype, int dest, int tag) {
    if (!initialized) throw std::runtime_error("MPI not initialized");
    MPI_Send(data, count, datatype, dest, tag, MPI_COMM_WORLD);
}
