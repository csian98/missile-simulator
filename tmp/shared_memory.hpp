#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <unistd.h>

#include <ctime>
#include <signal.h>
 
#define  KEY_NUM   8386
#define  MEM_SIZE  256
#define _SIGUSR1  10

void init_shared_memory(int& shmid);

void exit_shared_memory(int& shmid);

void write_shared_memory(int& shmid, char* wMemory, int size);

void read_shared_memory(int& shmid, char* rMemory, int size);

#endif
