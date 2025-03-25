#include "coord.hpp"

class vec_cart{
public:
  vec_cart(double x, double y, double z): x(x), y(y), z(z) {}
  
  vec_cart(const vec_cart&)=default;
  
  vec_cart& operator=(const vec_cart&)=default;
  
  virtual ~vec_cart() {}
  
  operator vec() const;
  
  vec_cart operator+(const vec_cart& other){
    return vec_cart(x+other.x, y+other.y, z+other.z);
  }
  double operator*(const vec_cart& other){
    return x*other.x+ y*other.y+ z*other.z;
  }
  double size(void) const{
    return std::sqrt(x*x+ y*y+ z*z);
  }
private:
  double x, y, z;
};

class vec{
public:
  friend class loc;
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
  double operator*(const vec& other){
    return static_cast<vec_cart>(*this)*static_cast<vec_cart>(other);
  }
  double size(void) const{
    return r;
  }
  double inter_degree(const vec& other){
    return std::acos( ((*this)*other)/( ((*this).size())*(other.size()) ) );
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
private:
  vec location;
};
