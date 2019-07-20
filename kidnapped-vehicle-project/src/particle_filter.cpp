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

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;
using namespace std;

default_random_engine randnum;

double multivariate_prob(double x, double y, double mu_x, double mu_y, double std_x, double std_y){
  double gaussian_norm, exponent, weight;
  gaussian_norm = 1/ (2 * M_PI * std_x * std_y);
  exponent = -(pow(x - mu_x, 2)/ (2 * pow(std_x, 2)) + pow(y - mu_y, 2)/ (2 * pow(std_y, 2)));
  weight = gaussian_norm * exp(exponent);
  
  return weight;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  cout << "init can" << endl << flush;
  num_particles = 500;  // TODO: Set the number of particles
  
  // Initialize x, y, theta std
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  
  // Normal distributions
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  // Generate initial particles with GPS coordinates with Gaussian Noise
  for (int i = 0; i < num_particles; i++){
    Particle P;
    P.id = i;
    P.x = dist_x(randnum);
    P.y = dist_y(randnum);
    P.theta = dist_theta(randnum);
    P.weight = 1.0;
    particles.push_back(P);
  }
  
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  cout << "prediction can" << endl << flush;
  double pred_x, pred_y, pred_theta;
  double std_pos_x = std_pos[0];
  double std_pos_y = std_pos[1];
  double std_pos_theta = std_pos[2];
  
  for (int i = 0; i < num_particles; i++){
    if (fabs(yaw_rate) > 0.0001) {
      pred_x = particles[i].x + (velocity/ yaw_rate) * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      pred_y = particles[i].y + (velocity/ yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      pred_theta = particles[i].theta + yaw_rate*delta_t;
    }
    
    else if (fabs(yaw_rate) < 0.0001) {
      pred_x = particles[i].x + (velocity * delta_t * cos(particles[i].theta));
      pred_y = particles[i].y + (velocity * delta_t * sin(particles[i].theta));
      pred_theta = particles[i].theta;
    }
    
    // Normal distributions
    normal_distribution<double> dist_x(pred_x, std_pos_x);
    normal_distribution<double> dist_y(pred_y, std_pos_y);
    normal_distribution<double> dist_theta(pred_theta, std_pos_theta);
      
    particles[i].x = dist_x(randnum);
    particles[i].y = dist_y(randnum);
    particles[i].theta = dist_theta(randnum);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  cout << "dataAssociation can" << endl << flush;
  for (unsigned int i = 0; i < observations.size(); i++){
    
    LandmarkObs observed = observations[i];
    double distance_min = numeric_limits<double>::max();
    
    for (unsigned int j = 0; j < predicted.size(); j++){
      
      double distance = dist(observed.x, observed.y, predicted[j].x, predicted[j].y);
      
      if (distance <= distance_min){
        distance_min = distance;
        observations[i].id = predicted[j].id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a multi-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  cout << "update can" << endl << flush;
  double std_landmark_x = std_landmark[0];
  double std_landmark_y = std_landmark[1];
  
  for (unsigned int i = 0; i < particles.size(); i++){
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

    // transforming car observations to particles coordinates system
    vector<LandmarkObs> transformed_observations;
    for (unsigned int j = 0; j < observations.size(); j++){
      LandmarkObs transformed;
      transformed.x = particle_x + (cos(particle_theta) * observations[i].x) - (sin(particle_theta) * observations[i].y);
      transformed.y = particle_y + (sin(particle_theta) * observations[i].x) + (cos(particle_theta) * observations[i].y);
      transformed.id = j;
      transformed_observations.push_back(transformed);
    }
    
    // remove landmarks that are out of sensor range
    vector<LandmarkObs> filtered_landmarks;
    for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++){
      double distance;
      distance = dist(particle_x, particle_y, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
      if (distance < sensor_range){
        LandmarkObs landmark;
        landmark.x = map_landmarks.landmark_list[k].x_f;
        landmark.y = map_landmarks.landmark_list[k].y_f;
        landmark.id = map_landmarks.landmark_list[k].id_i;
        filtered_landmarks.push_back(landmark);
      }
    }
    // sorting transformed observations into their group of nearest landmarks neighbour
    dataAssociation(filtered_landmarks, transformed_observations);
    
    // calculate particle weights using multi-variate gaussian
    weights.clear();
    double weight = 1.0;
    double t_x, t_y, l_x, l_y, observation_weight;
    int t_id;
    for (unsigned int h = 0; h < transformed_observations.size(); h++){
      t_x = transformed_observations[h].x;
      t_y = transformed_observations[h].y;
      t_id = transformed_observations[h].id;
      
      for (unsigned int g = 0; g < filtered_landmarks.size(); i++){
        int l_id = filtered_landmarks[g].id;
        // check if both id of landmark and observation match
        if (l_id == t_id){
          l_x = filtered_landmarks[g].x;
          l_y = filtered_landmarks[g].y;
        }
      }
      // calculate weight for that observation
      observation_weight = multivariate_prob(t_x, t_y, l_x, l_y, std_landmark_x, std_landmark_y);
      // multiply with previous accumulated weights
      weight *= observation_weight;
    }
    // Make product of all weights to be particle's weight
    particles[i].weight = weight;
    weights.push_back(weight);
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  cout << "resample can" << endl << flush;
  uniform_int_distribution<int> rand_index(0, num_particles - 1);
  int index = rand_index(randnum);
  double beta = 0.0;
  double max_weight = 2 ** max_element(weights.begin(), weights.end());
           
  vector<Particle> resampled_particles;
  for (unsigned int i = 0; i < particles.size(); i++){
    uniform_real_distribution<double> random_weight(0.0, max_weight);
    beta += random_weight(randnum);
    while (beta > particles[index].weight){
      beta -= particles[index].weight;
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }
  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  cout << "SetAssociations can" << endl << flush;
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  cout << "getAssociations can" << endl;
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  cout << "getSenseCoord can" << endl << flush;
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}