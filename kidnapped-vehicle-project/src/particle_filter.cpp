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
using std::cout;
using std::endl;
using namespace std;

double multivariate_prob(double x, double y, double mu_x, double mu_y, double std_x, double std_y){
  double gaussian_norm = 1/ (2 * M_PI * std_x * std_y);
  double exponent = -((pow(x - mu_x, 2)/ (2 * pow(std_x, 2))) + (pow(y - mu_y, 2)/ (2 * pow(std_y, 2))));
  double weight = gaussian_norm * exp(exponent);
  return weight;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
  num_particles = 100;  // TODO: Set the number of particles
  default_random_engine randnum;
  // Normal distributed x, y, theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  // Initialize particles and weights
  for (int i = 0; i < num_particles; i++){
    Particle P;
    P.id = i;
    P.x = dist_x(randnum);
    P.y = dist_y(randnum);
    P.theta = dist_theta(randnum);
    P.weight = 1.0;
    particles.push_back(P);
    weights.push_back(P.weight);
  }
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  default_random_engine randnum;
  // Iterate through particles
  for (auto& p: particles){
    // If there is change of angle
    if (fabs(yaw_rate) == 0) {
      p.x += velocity*delta_t*cos(p.theta);
      p.y += velocity*delta_t*sin(p.theta);
    }
    else {
      p.x += (velocity/yaw_rate)*(sin(p.theta + yaw_rate*delta_t) - sin(p.theta));
      p.y += (velocity/yaw_rate)*(cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
      p.theta += yaw_rate*delta_t;
    }
    // Normal distributed x, y, theta
    normal_distribution<double> dist_x(p.x, std_pos[0]);
    normal_distribution<double> dist_y(p.y, std_pos[1]);
    normal_distribution<double> dist_theta(p.theta, std_pos[2]);
    // Add gaussian noise to x, y, theta
    p.x = dist_x(randnum);
    p.y = dist_y(randnum);
    p.theta = dist_theta(randnum);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  for (auto& obs: observations){
    double distance_min = numeric_limits<double>::max();

    for (const auto& pred: predicted){
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      if (distance < distance_min){
        distance_min = distance;
        obs.id = pred.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  // Clearing all the previous weights 
  weights.clear();
  for (auto& particle: particles){
    particle.weight = 1.0;
    // Step 1: collect valid landmarks
    vector<LandmarkObs> predicted_landmarks;
    for (const auto& landmark: map_landmarks.landmark_list){
      double distance = dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
      if (distance < sensor_range){
        LandmarkObs valid_landmark = LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f};
        predicted_landmarks.push_back(valid_landmark);
      }
    }

    // Step 2: transform landmark observations from vehicle to map coordinate
    vector<LandmarkObs> transformed_observations;
    for (const auto& obs: observations){
      LandmarkObs transformed;
      transformed.x = particle.x + (obs.x*cos(particle.theta)) - (obs.y*sin(particle.theta));
      transformed.y = particle.y + (obs.x*sin(particle.theta)) + (obs.y*cos(particle.theta));
      transformed.id = obs.id;
      transformed_observations.push_back(transformed);
    }

    // Step 3: match predicted and transformed landmarks index
    dataAssociation(predicted_landmarks, transformed_observations);

    // Step 4: calculate weights
    for (const auto& obs: transformed_observations){
      for (const auto& lm: predicted_landmarks){
        if (obs.id ==  lm.id){
          double mvp = multivariate_prob(obs.x, obs.y, lm.x, lm.y, std_landmark[0], std_landmark[1]);
          if (mvp == 0){
            mvp = 0.001;
          }
          particle.weight *= mvp;
        }
      }
    }
    weights.push_back(particle.weight);
  }
}

void ParticleFilter::resample() {
  
  default_random_engine randnum;
  uniform_int_distribution<int> rand_index(0, num_particles - 1);
  int index = rand_index(randnum);
  double beta = 0.0;
  double max_weight = 2 ** max_element(weights.begin(), weights.end());
  uniform_real_distribution<double> random_weight(0.0, max_weight);

  vector<Particle> resampled_particles;
  for (unsigned int i = 0; i < particles.size(); i++){
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
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
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