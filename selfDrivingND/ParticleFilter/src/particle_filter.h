
/**
 * particle_filter.h
 * 2D particle filter class.
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include <limits>
#include <iostream>
#include "Particle.h"
#include "helper_functions.h"

class ParticleFilter
{
public:
  // Constructor
  // @param num_particles Number of particles
  ParticleFilter() : num_particles(0), is_initialized(false) {}

  // Destructor
  ~ParticleFilter() {}

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], 
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], 
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_pos[], double velocity,
                  double yaw_rate);

  /**
   * dataAssociation Finds which observations correspond to which landmarks 
   *   (likely by using a nearest-neighbors data association).
   * @param predicted Vector of predicted landmark observations
   * @param observations Vector of landmark observations
   */
  void dataAssociation(std::vector<LandmarkObs> predicted,
                       std::vector<LandmarkObs> &observations);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   *   of the observed measurements. 
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double std_landmark[],
                     const std::vector<LandmarkObs> &observations,
                     const Map &map_landmarks);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();

  /**
   * Set a particles list of associations, along with the associations'
   *   calculated world x,y coordinates
   * This can be a very useful debugging tool to make sure transformations 
   *   are correct and assocations correctly connected
   */
  void SetAssociations(Particle &particle, const std::vector<int> &associations,
                       const std::vector<double> &sense_x,
                       const std::vector<double> &sense_y);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const
  {
    return is_initialized;
  }

  void printFilter()
  {
    for (auto p : particles)
    {
      p.printParticle();
    }
  }

  /**
   * Used for obtaining debugging information related to particles.
   */
  std::string getAssociations(Particle best);
  std::string getSenseCoord(Particle best, std::string coord);

  // Set of current particles
  std::vector<Particle> particles;

  std::vector<double> getWeights()
  {
    return weights;
  }

private:
  // Number of particles to draw
  int num_particles;

  // Flag, if filter is initialized
  bool is_initialized;

  // Vector of weights of all particles
  std::vector<double> weights;

  /*
  * Used to transform observations in the car frame to map frame for a given particle
  * @param p - Particle for which observations needs to be transformed
  * @param observations - all observations in the car frame from sensors
  * 
  */
  void carToMapFrame(Particle &p,
                     const std::vector<LandmarkObs> &observations);
  /*
* Identify the closest landmark to each observation
* @param observations_map - vector of observations in the map frame
* @param map_landmarks - map object
* @param associations -  vector of nearest landmarks ids to be filled up
* @param sense_x - vector of x positions of the nearest landmark positions to be filled up
* @param sense_y - vector of y positions of the nearest landmark position to be filled up
*/

  void findNearestLandmark(Particle &p,
                           const Map &map_landmarks);

  /*
* Used to get probability for observations for a given particle and to set its weight
* @param observations_map observations in the map frame
* @param p with updated associated landmark for each observation
*/

  void setWeight(Particle &p,
                 const Map &map_landmark,
                 double std_landmark[]);

  void normalizeWeight();

  void clearStaleData(Particle &p);
};

#endif // PARTICLE_FILTER_H_