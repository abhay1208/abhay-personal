/**
 * Particle.h
 * Particle class for particle filter.
 *
 * Created on: Sep 30, 2020
 * Author: Abhay Gupta
 */
#ifndef PARTICLE_H_
#define PARTICLE_H_

#include <vector>
#include <iostream>
#include <math.h>
#include "helper_functions.h"

class Particle
{
public:
    int id;
    double x;
    double y;
    double theta;
    double weight;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    /**
     * Motion model to update particle position
     * @param dT Time Delta
     * @param std_pos vector of std dev in (x, y, theta)
     * @param velocity - constant velocity for dT
     * @param yaw_rate - constant yaw rate for dT
     */

    void updatePosition(double dT, double std_pos[], double velocity, double yaw_rate);

    /** Print Particle Informaton
     */

    void printParticle() const;

    /**
     * PDF for a multivariable gaussian distribution for zero covariance
     * @param mu_x mean in x
     * @param mu_y mean in y
     * @param sig_x std_Dev in x
     * @param sig_y std_Dev in y
     * @param x_obs - x observation for probabily density needs to be calculated
     * @param y_obs - y observation for which probability density needs to be calculated
     * @output - probability density for x_obs and y_obs
     */
    static double pdfMultVar(double mu_x, double mu_y, double sig_x, double sig_y, double x_obs, double y_obs);
};
#endif // PARTICLE_FILTER_H_