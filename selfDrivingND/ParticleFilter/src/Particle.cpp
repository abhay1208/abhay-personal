#include "Particle.h"

/**
     * Motion model to update particle position
     * @param dT Time Delta
     * @param std_pos vector of std dev in (x, y, theta)
     * @param velocity - constant velocity for dT
     * @param yaw_rate - constant yaw rate for dT
     */

void Particle::updatePosition(double dT, double std_pos[], double velocity, double yaw_rate)
{

    if (abs(yaw_rate) < 1e-5)
    {
        x += velocity * dT * cos(theta);
        y += velocity * dT * sin(theta);
    }
    else
    {
        x += (velocity / yaw_rate) * (sin(theta + yaw_rate * dT) - sin(theta));
        y += (velocity / yaw_rate) * (cos(theta) - cos(theta + yaw_rate * dT));
        theta += yaw_rate * dT;
    }

    // Adding noise to prediction
    x = getGaussianSample(x, std_pos[0]);
    y = getGaussianSample(y, std_pos[1]);
    theta = getGaussianSample(theta, std_pos[2]);
}

/** Print Particle Informaton
     */

void Particle::printParticle() const
{

    std::cout << "ID : " << id << "\t"
              << "X: " << x << "\t"
              << "Y: " << y << "\t"
              << "Theta: " << theta << "\t"
              << "Weight: " << weight << std::endl;
}

/**
 * Assumed a multivariate gaussian distribution and return probability of (x,y) given mean and std dev of the distrbution
 */

double Particle::pdfMultVar(double mu_x, double mu_y, double sig_x, double sig_y, double x_obs, double y_obs)
{
    // calculate normalization term
    double gauss_norm;
    gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // // calculate exponent
    double exponent;
    exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2))) + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    // calculate weight using normalization terms and exponent
    double weight;
    weight = gauss_norm * exp(-exponent);

    return weight;
}