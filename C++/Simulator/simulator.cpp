#include "simulator.hpp"

namespace mpcc
{

  Simulator(Car car, Tire tire, ArcLengthSpline centerLine): d_car(car), d_tire(tire), d_models(d_car, d_tire), d_centerLine(centerLine){

  }

  

} // namespace mpcc