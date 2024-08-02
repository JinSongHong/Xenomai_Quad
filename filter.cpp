#include "filter.h"
#include <data_mutex.h>

filter::filter()
{

}

double filter::tustin_derivate(double *u, double *y, double cut_off)
{
  double tau = 1/(2*M_PI*cut_off);
  
    return (-2*u[1]+2*u[0] - (T-2*tau)*y[1]) / (T+2*tau);

}

double filter::LPF1(double *u, double *y, double tau)
{
    return (T*(u[0]+u[1])+(2*tau-T)*y[1])/(2*tau+T);
}

double filter::lowpassfilter(double u, double u_old, double y_old, double cut_off)
{
  double tau = 1/(2*M_PI*cut_off);
  
  
  return (T*(u + u_old)+(2*tau-T) * y_old) / (2*tau+T);
}
