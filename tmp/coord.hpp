#ifndef COORD_HPP
#define COORD_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <memory>

static const double PI=M_PI;

static const double EARTH_R=6371000;

class vec;

class vec_cart{
public:
  vec_cart(double, double, double);
  
  vec_cart(const vec_cart&);
  
  vec_cart& operator=(const vec_cart&);
  
  virtual ~vec_cart();
  
  operator vec() const;
  
  vec_cart operator+(const vec_cart&);
  
  double operator*(const vec_cart&);
  
  double size(void) const;
private:
  double x, y, z;
};

class vec{
public:
  friend class loc;
  
  vec(double, double, double);
  
  vec(const vec&);
  
  vec& operator=(const vec&);
  
  virtual ~vec();
  
  operator vec_cart() const;
  
  vec operator+(const vec&);
  
  double operator*(const vec&);
  
  double size(void) const;
  
  double inter_degree(const vec&);
private:
  double r, theta, phi;
};

class loc{
public:
  loc(double lati, double longi, double heig=0);
  
  virtual void print_loc(void);
  
  virtual ~loc();
  
  void move(const vec&);
private:
  vec location;
};

#endif
