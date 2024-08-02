#ifndef FILTER_H
#define FILTER_H

class filter
{
private:
    double output;
    double T = 0.001;

public:

    filter();
    double tustin_derivate(double *u, double *y, double tau);
    double LPF1(double *u, double *y, double tau);
    double lowpassfilter(double u, double u_old, double y_old, double tau);
};

#endif // FILTER_H
