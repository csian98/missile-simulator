#include "missile.hpp"
#include "coord.hpp"
#include "shared_memory.hpp"

int main(void){
  int shmid;
  
  vec_cart loc=vec_cart(0, 0, 0);
  vec_cart destination=vec_cart(100000, -50000, 10000);
  missile m1=missile(loc, destination);
  
  
  
  return 0;
}
