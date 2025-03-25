#include "missile.hpp"

int main(){
  int shmid;
  char state='1';
  pid_t pid_py;
  char* shm_buf=(char*)malloc(sizeof(char)*MEM_SIZE);
  char* buf=(char*)malloc(sizeof(char)*MEM_SIZE);
  
  init_shared_memory(shmid);
  
  if((pid_py=fork())==-1){
    perror("fork");
    exit(1);
  }
  
  if(pid_py==0){
    execl("./plot.py", (char*)0);
    perror("execl");
    exit(1);
  }
  
  vec_cart loc=vec_cart(0, 0, 0);
  vec_cart destination=vec_cart(100000, 100000, 100000);
  
  vec_cart init_dir=vec_cart(0, 0, 1);
  
  missile m1=missile(loc, destination);
  m1.state_format(shm_buf);
  write_shared_memory(shmid, shm_buf, MEM_SIZE);
  
  m1.set_dir(init_dir);
  
  while(m1.state()){
    m1.step();
    m1.state_format(shm_buf);
    
    write_shared_memory(shmid, shm_buf, MEM_SIZE);
    read_shared_memory(shmid, shm_buf, MEM_SIZE);
    while(shm_buf[0]!='w'){
      read_shared_memory(shmid, shm_buf, MEM_SIZE);
    }
  }
  m1.state_format(shm_buf);
  write_shared_memory(shmid, shm_buf, MEM_SIZE);
  free(shm_buf); free(buf);
  
  exit_shared_memory(shmid);
  return 0;
}
