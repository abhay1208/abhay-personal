/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <unordered_set>

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 10;
  for (int i = 0; i < num_particles; ++i)
  {
    // Create each particle using normal distribution
    Particle p;
    p.id = i;
    p.x = getGaussianSample(x, std[0]);
    p.y = getGaussianSample(y, std[1]);
    p.theta = getGaussianSample(theta, std[2]);
    p.weight = 1.0 / num_particles; // Assume uniform distribution

    // Fill up particles vector
    particles.push_back(p);
    weights.push_back(p.weight);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  // Update each particle position
  for (Particle &p : particles)
  {
    p.updatePosition(delta_t, std_pos, velocity, yaw_rate);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /*
  Iterate through each particle
  Transform observations in the map frame
  Find nearest neighbor for each observation
  Associate nearest landmark with each observation
  Use multivariate gaussian distrobution to get probability for each associated landmark
  Multiply probability for each observation and assign it to weight of each particle
  */

  // Empty the weights vector
  weights.clear();
  for (auto &p : particles)
  {
    clearStaleData(p);
    carToMapFrame(p, observations);
    findNearestLandmark(p, map_landmarks);
    setWeight(p, map_landmarks, std_landmark);
  }
}

void ParticleFilter::resample()
{
  // We use resampling wheel technique for resampling
  vector<Particle> resampled_particles;
  int index = generateRandomNum(num_particles - 1);
  double beta = 0.;
  double mw = *(std::max_element(weights.begin(), weights.end()));

  for (int i = 0; i < num_particles; ++i)
  {
    double random_number = ((double)rand() / (RAND_MAX));
    beta += random_number * 2.0 * mw;
    while (beta > weights[index])
    {
      beta -= weights[index];
      index = (index + 1) % (num_particles);
    }
    // Regenerate new ids for the particles
    particles[index].id = i;
    resampled_particles.push_back(particles[index]);
  }

  // Assign the new set of particles
  particles = resampled_particles;
}

void ParticleFilter::clearStaleData(Particle &p)
{
  p.sense_x.clear();
  p.sense_y.clear();
  p.associations.clear();
}

void ParticleFilter::carToMapFrame(Particle &p,
                                   const std::vector<LandmarkObs> &observations)
{
  for (auto obs : observations)
  {
    double x = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta);
    double y = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
    p.sense_x.push_back(x);
    p.sense_y.push_back(y);
  }
}

void ParticleFilter::setWeight(Particle &p, const Map &map_landmark, double std_landmark[])
{
  if (p.associations.empty())
  {
    std::cout << "No associations" << std::endl;
    return; // Return without updating the weights
  }

  double prob = 1.;
  for (int i = 0; i < p.associations.size(); ++i)
  {
    int id = p.associations[i];
    auto landmark = map_landmark.landmark_list[id - 1]; //Map landmars start with 1 instead of 0
    double each_prob = Particle::pdfMultVar(landmark.x_f, landmark.y_f, std_landmark[0], std_landmark[1], p.sense_x[i], p.sense_y[i]);
    // std::cout << "(Exp_X, Exp_Y) \t (" << landmark.x_f << ", " << landmark.y_f << ") \t (Obs_X, Obs_Y) \t(" << p.sense_x[i] << ", " << p.sense_y[i] << ")\t Prob = " << each_prob << std::endl;
    prob *= each_prob;
  }
  p.weight = prob;
  weights.push_back(p.weight);
}

void ParticleFilter::findNearestLandmark(Particle &p, const Map &map_landmarks)
{
  int landmark_id;
  for (int i = 0; i < p.sense_x.size(); ++i)
  {
    double minimum_distance = std::numeric_limits<double>::max();
    for (auto landmark : map_landmarks.landmark_list)
    {
      double current_distance = dist(p.sense_x[i], p.sense_y[i], landmark.x_f, landmark.y_f);
      if (current_distance < minimum_distance)
      {
        landmark_id = landmark.id_i;
        minimum_distance = current_distance;
      }
    }
    p.associations.push_back(landmark_id);
  }
}

void ParticleFilter::normalizeWeight()
{
  double total_weight = 0;
  for (int i = 0; i < particles.size(); ++i)
  {
    total_weight += particles[i].weight;
  }

  for (int i = 0; i < particles.size(); ++i)
  {
    double normalized_weight = particles[i].weight / total_weight;
    particles[i].weight = normalized_weight;
    weights[i] = normalized_weight;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}
