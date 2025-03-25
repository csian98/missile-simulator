#include "shared_memory.hpp"

void init_shared_memory(int& shmid);
void exit_shared_memory(int& shmid);
void write_shared_memory(int& shmid, char* wMemory, int size);
void read_shared_memory(int& shmid, char* rMemory, int size);

void init_shared_memory(int& shmid){
  if((shmid=shmget((key_t)KEY_NUM, MEM_SIZE, IPC_CREAT| IPC_EXCL| 0666))==-1){
    std::cout<<"Already Shared_memory created"<<std::endl;
    
    shmid=shmget((key_t)KEY_NUM, MEM_SIZE, IPC_CREAT| 0666);
    
    if(shmid==-1){
      perror("shmget");
      exit(1);
    } else {
      exit_shared_memory(shmid);
      shmid=shmget((key_t)KEY_NUM, MEM_SIZE, IPC_CREAT| 0666);
      if(shmid==-1){
        perror("shmget");
        exit(1);
      }
    }
  }
  std::cout<<"Init shared memory"<<std::endl;
}

void exit_shared_memory(int& shmid){
  if(shmctl(shmid, IPC_RMID, 0)==-1){
    perror("shmctl");
    exit(1);
  }
  std::cout<<"Exit shared memory"<<std::endl;
}

void write_shared_memory(int& shmid, char* wMemory, int size){
  void* shmaddr;
  if(size> MEM_SIZE){
    perror("Shared memory size over");
    exit(1);
  }
  if((shmaddr=shmat(shmid, (void*)0, 0))==(void*)-1){
    perror("shmat");
    exit(1);
  }
  
  memcpy((char*)shmaddr, wMemory, size);
  
  if(shmdt(shmaddr)==-1){
    perror("shmdat");
    exit(1);
  }
}

void read_shared_memory(int& shmid, char* rMemory, int size){
  void* shmaddr;
  char tmp[MEM_SIZE]={0};
  
  if((shmaddr=shmat(shmid, (void*)0, 0))==(void*)-1){
    perror("shmat");
    exit(1);
  }
  
  memcpy(rMemory, (char*)shmaddr, size);
  
  if(shmdt(shmaddr)==-1){
    perror("shmdt");
    exit(1);
  }
}
