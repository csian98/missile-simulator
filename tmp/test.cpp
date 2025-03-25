#include <iostream>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <ctime>

#define KEY_NUM   8386
#define MEM_SIZE  1024

static const double PI=M_PI;

static const double EARTH_R=6371000;

class vec;

class vec_cart{
public:
  friend class missile;
  vec_cart(double x, double y, double z): x(x), y(y), z(z) {}
  virtual ~vec_cart() {}
  
  operator vec() const;
  
  vec_cart operator+(const vec_cart& other){
    return vec_cart(x+other.x, y+other.y, z+other.z);
  }
  vec_cart operator-(const vec_cart& other){
    return vec_cart(x-other.x, y-other.y, z-other.z);
  }
  double operator*(const vec_cart& other){
    return x*other.x+ y*other.y+ z*other.z;
  }
  double size(void) const{
    return std::sqrt(x*x+ y*y+ z*z);
  }
  void print_vec(void){
    std::cout<<"x: "<<x<<", y: "<<y<<", z: "<<z<<std::endl;
  }
private:
  double x, y, z;
};

class vec{
public:
  friend class loc;
  friend class missile;
  
  vec(double r, double theta, double phi): r(r), theta(theta), phi(phi) {}
  
  vec(const vec&)=default;
  
  vec& operator=(const vec&)=default;
  
  virtual ~vec() {}
  
  operator vec_cart() const{
    double x, y, z;
    x=r*std::sin(theta)*std::cos(phi);
    y=r*std::sin(theta)*std::sin(phi);
    z=r*std::cos(theta);
    return vec_cart(x, y, z);
  }
  
  vec operator+(const vec& other){
    return static_cast<vec>(static_cast<vec_cart>(*this)+static_cast<vec_cart>(other));
  }
  vec operator-(const vec& other){
    return static_cast<vec>(static_cast<vec_cart>(*this)-static_cast<vec_cart>(other));
  }
  double operator*(const vec& other){
    return static_cast<vec_cart>(*this)*static_cast<vec_cart>(other);
  }
  double size(void) const{
    return r;
  }
  double inter_degree(const vec& other){
    return (std::acos( ((*this)*other)/( ((*this).size())*(other.size()) ) ))/PI*180;
  }
  void print_vec(void){
    std::cout<<"r: "<<r<<", theta: "<<theta<<", phi: "<<phi<<std::endl;
  }
  
  void set_size(double tr){
    r=tr;
  }
private:
  double r, theta, phi;
};

vec_cart::operator vec() const{
  double r, theta=PI/2, phi;
  r=std::sqrt(x*x+ y*y+ z*z);
  if(z!=0) theta=std::atan(std::sqrt(x*x+ y*y)/ z);
  if(x!=0) phi=x>0? std::atan(y/x): M_PI+std::atan(y/x);
  else phi=y>0? PI/2: -PI/2;
  return vec(r, theta, phi);
}

class loc{
public:
  friend class missile;
  loc(double lati, double longi, double heig=0): location(EARTH_R+heig, PI/2-(lati/180*PI), longi/180*PI) {}
  
  virtual void print_loc(void){
    double lati=(PI/2-(location.theta))*180/PI;
    double longi=(location.phi)*180/PI;
    double height=(location.r)-EARTH_R;
    
    bool is_N=lati>0? true: false;
    bool is_E=longi>0? true: false;
    
    std::cout<<lati<<(is_N==true? "N": "S")<<" : "<<longi<<(is_E==true? "E": "W")<<" Height: "<<height<<std::endl;
  }
  virtual ~loc() {}
  
  void move(const vec& change){
    location=location+change;
  }
  
  vec get_dir(void){
    return vec(1, location.theta, location.phi);
  }
  vec inter_loc_dir(loc& other){
    vec temp(other.location-location);
    temp.set_size(1);
    return temp;
  }
private:
  vec location;
};

class missile{
public:
  missile(vec_cart& location, vec_cart& dest):
    loc(vec_cart(location)), dest(vec_cart(dest)), velo(vec_cart(0, 0, 0)), acc(vec_cart(0, 0, 0)), dir(vec_cart(0, 0, 1)), t_dir(vec_cart(0, 0, -1)) {
      fuel=100;
      mass=900;
      std::cout<<"MISSILE READY"<<std::endl;
    }
  virtual ~missile() {}
  
  void state_format(char* shm_buf){
    memset(shm_buf, 0, MEM_SIZE);
    snprintf(shm_buf, MEM_SIZE, "%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f", loc.x, loc.y, loc.z, dest.x, dest.y, dest.z, velo.x, velo.y, velo.z, acc.x, acc.y, acc.z, dir.x, dir.y, dir.z, t_dir.x, t_dir.y, t_dir.z, fuel);
  }
  
private:
  vec_cart loc, dest, velo, acc, dir, t_dir;;
  double fuel, mass;
};

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

int main(void){
  int shmid, fd[2];
  char state='1';
  pid_t pid_py;
  char* shm_buf=(char*)malloc(sizeof(char)*MEM_SIZE);
  
  init_shared_memory(shmid);
  
  if(pipe(fd)<0){
    perror("pipe");
    exit(1);
  }
  
  if((pid_py=fork())==-1){
    perror("fork");
    exit(1);
  }
  
  if(pid_py==0){
    close(STDIN_FILENO);
    dup(fd[0]); close(fd[0]);
    
    execl("./plot.py", (char*)0);
    perror("execl");
    exit(1);
  }
  
  vec_cart loc=vec_cart(0, 0, 0);
  vec_cart destination=vec_cart(100000, -50000, 10000);
  
  missile m1=missile(loc, destination);
  
  m1.state_format(shm_buf);
  write_shared_memory(shmid, shm_buf, MEM_SIZE);
  
  sleep(3);

  write(fd[1], &state, sizeof(char));

  free(shm_buf);
  
  sleep(8);
  
  exit_shared_memory(shmid);
  kill(pid_py, 9);
  return 0;
}
