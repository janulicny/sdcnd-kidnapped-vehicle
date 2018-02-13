/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // TODO: Create normal distributions for x, y and theta
  default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
  
  // Set number of particles
  num_particles = 100;
  
  // Initialize the particles according to the distribution of the initial GPS measurement
  for (int i = 0; i < num_particles; ++i) {
		Particle p;
		p.id = i;
    p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);    
		is_initialized = true;
		// Print your samples to the terminal.
		// cout << "Sample " << i + 1 << " " << p.x << " " << p.y << " " << p.theta << endl;
	}
  

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  // Create normal distributions for x, y and theta motion updates
  default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
  for (int i = 0; i < num_particles; ++i) {
    // motion update if yawrate is 0
    if (yaw_rate==0) {
      particles[i].x += velocity*delta_t*cos(particles[i].theta);
      particles[i].y += velocity*delta_t*sin(particles[i].theta);
    }
    // motion update with nonzero yawrate
    else {
    particles[i].x += (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
    particles[i].y += (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
    particles[i].theta += + yaw_rate*delta_t;
    }
    // add Gaussian noise to the motion
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);  
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  // decided not to use this
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  // go through all particles
  for (int i = 0; i < num_particles; ++i) {
    // predict what landmarks will be in the particle's sensor range
    vector<LandmarkObs> predicted;
    // delete old landmark associations
    particles[i].associations.clear();
    particles[i].sense_x.clear();
    particles[i].sense_y.clear();
    for (int j=0; j < map_landmarks.landmark_list.size(); ++j) {
      if (dist(map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f, particles[i].x, particles[i].y) < sensor_range) {
        predicted.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f });
        particles[i].associations.push_back(map_landmarks.landmark_list[j].id_i);
        particles[i].sense_x.push_back(map_landmarks.landmark_list[j].x_f);
        particles[i].sense_y.push_back(map_landmarks.landmark_list[j].y_f);
      }
    }
    
    
    // reset the weight
    particles[i].weight = 1.0;
    
    for (int j=0; j <observations.size(); ++j) {
      // transform the observation from vehicle coordinates to map coordinates
      float obs_map_x = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      float obs_map_y = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      // landmark association - go through predicted landmarks and find the closest one
      float min_dist = 1000000.0;
      float closest_landmark_x = 1000000.0;
      float closest_landmark_y = 1000000.0; 
      for (int k=0; k < predicted.size(); ++k) {
        // predicted landmark position in map coordinates
        float landmark_x = predicted[k].x;
        float landmark_y = predicted[k].y;
        float distance = dist(landmark_x, landmark_y, obs_map_x, obs_map_y);
        if (distance < min_dist) {
          min_dist = distance;
          closest_landmark_x = landmark_x;
          closest_landmark_y = landmark_y;
        } 
      }
      // calculate the weight contribution from single observation
      double weight_update = 1/(2*M_PI*std_landmark[0]*std_landmark[1])*exp(-(pow(closest_landmark_x-obs_map_x,2)/(2*pow(std_landmark[0], 2)) + pow(closest_landmark_y-obs_map_y,2)/(2*pow(std_landmark[1], 2))));
      // update the weight
      particles[i].weight *= weight_update;
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  // find the max value of weight among particles
  float max_weight = 0;
  for (int i=0; i<num_particles; ++i) {
    if (particles[i].weight>max_weight) {
      max_weight = particles[i].weight;
    }
  }
  
  // prepare to store resampled particles
  vector<Particle> resampled_particles;
  
  // create uniform distribution
  default_random_engine gen;
  uniform_real_distribution<double> distribution(0.0,1.0);
  // randomly choose starting index
  int index = distribution(gen)*(num_particles-1);
  float beta = 0;
  // implementation of resample wheel
  for (int i=0; i<num_particles; ++i) {
    beta += 2*max_weight*distribution(gen);
    while (particles[index].weight<beta) {
      beta -= particles[index].weight;
      index = (index+1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }
  // assign the resampled particles to the filter
  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
