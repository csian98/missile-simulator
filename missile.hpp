#ifndef MISSILE_HPP
#define MISSILE_HPP

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
  
  vec_cart(const vec_cart&)=default;
  
  vec_cart& operator=(const vec_cart&)=default;
  
  operator vec() const;
  
  vec_cart operator+(const vec_cart& other){
    return vec_cart(x+other.x, y+other.y, z+other.z);
  }
  vec_cart operator-(const vec_cart& other){
    return vec_cart(x-other.x, y-other.y, z-other.z);
  }
  vec_cart operator*(const double num){
    return vec_cart(x*num, y*num, z*num);
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
  void make_unit(void){
    double size=std::sqrt(x*x+ y*y+ z*z);
    x=x/size; y=y/size; z=z/size;
  }
  double inter_degree(const vec_cart& other){
    return (std::acos( ((*this)*other)/( ((*this).size())*(other.size()) ) ))/PI*180;
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

// check_point
class missile{
public:
  missile(vec_cart& location, vec_cart& dest):
    loc(vec_cart(location)), dest(vec_cart(dest)), velo(vec_cart(0, 0, 0)), acc(vec_cart(0, 0, 0)), dir(vec_cart(0, 0, 1)), t_dir(vec(1, PI, 0)) {
      state_v=1; thru=124000; fuel=100;
      mass=900; f_mass=440; dm_dt=f_mass/(t_tot/t_sec);
      f_mass_i=f_mass;
      std::cout<<"MISSILE READY"<<std::endl;
  }
    
  virtual ~missile() {}
  
  vec_cart thru_rotation(void){
    double theta_x, theta_y;
    vec_cart z_ax=vec_cart(0, 0, 1);
    theta_x=-z_ax.inter_degree(dir-vec_cart(dir.x, 0, 0))*PI/180;
    theta_y=z_ax.inter_degree(vec_cart(dir.x, 0, std::sqrt(dir.y*dir.y+ dir.z*dir.z)))*PI/180;
    vec_cart t_dir_car=static_cast<vec_cart>(t_dir);
    
    vec_cart tmp=vec_cart(vec_cart(std::cos(theta_y), 0, std::sin(theta_y))*t_dir_car, vec_cart(0, 1, 0)*t_dir_car, vec_cart(-std::sin(theta_y), 0, std::cos(theta_y))*t_dir_car);
    return vec_cart(vec_cart(1, 0, 0)*tmp, vec_cart(0, std::cos(theta_x), -std::sin(theta_x))*tmp, vec_cart(0, std::sin(theta_x), std::cos(theta_x))*tmp);
  }
  
  vec_cart Force(void){
    vec_cart Ft=thru_rotation()*-thru;
    vec_cart Fg=vec_cart(0, 0, -1)*G*mass;
    vec_cart Fd=vec_cart(0, 0, 0);
    
    if(velo.size()!=0){
      Fd=velo*(1/velo.size())*(-0.5)*air_rho*Cd*velo.size()*velo.size()*0.1;
    }
    return Ft+Fg+Fd;
  }
  
  void step(void){
    if(f_mass<0){
      thru=0; dm_dt=0;
    }
    acc=Force()*(1/mass);
    velo=velo+acc*t_sec;
    loc=loc+velo*t_sec;

    mass-=dm_dt; f_mass-=dm_dt;
    fuel=f_mass/f_mass_i*100;
  }
  
  void set_dir(vec_cart other){
    dir=other;
    dir.make_unit();
  }
  
  int state(void){
    if(loc.z<-1){
      fuel=-100;
      state_v=0;
      return state_v;
    } else {
      return state_v;
    }
  }
  
  void state_format(char* shm_buf){
    memset(shm_buf, 0, MEM_SIZE);
    snprintf(shm_buf, MEM_SIZE, "%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f", loc.x, loc.y, loc.z, dest.x, dest.y, dest.z, velo.x, velo.y, velo.z, acc.x, acc.y, acc.z, dir.x, dir.y, dir.z, t_dir.r, t_dir.theta, t_dir.phi, fuel);
  }
  
  void print_state(void){
    std::cout<<"loc:"<<std::endl;
    loc.print_vec();
    std::cout<<"velo:"<<std::endl;
    velo.print_vec();
    std::cout<<"acc:"<<std::endl;
    acc.print_vec();
    std::cout<<"dir:"<<std::endl;
    dir.print_vec();
    std::cout<<"t_dir:"<<std::endl;
    t_dir.print_vec();
  }
  
private:
  vec_cart loc, dest, velo, acc, dir;
  vec t_dir;
  int state_v;
  double fuel, mass, f_mass, f_mass_i, thru, dm_dt;
  const double G=9.80665, Cd=0.3, air_rho=1.225, t_sec=0.1, t_tot=9;
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

#endif
