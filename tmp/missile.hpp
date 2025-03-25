#ifndef MISSILE_HPP
#define MISSILE_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <memory>
#include "coord.hpp"
#include "shared_memory.hpp"

class missile{
public:
  missile(vec_cart& location, vec_cart& dest){}
  
private:
  vec_cart loc, dest;
  vec velo, acc, dir, t_dir;
  double fuel, mass;
}

#endif
