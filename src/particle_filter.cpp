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
using std::uniform_real_distribution;
using std::uniform_int_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles    = 20;  // TODO: Set the number of particles
  
  std::default_random_engine gen;
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  for (int i=0 ; i<num_particles; i++) {
    
    
    Particle p;
    
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;
    
    
    particles.push_back(p);
      
  }
  
  is_initialized=true; 

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
   std::cout<<" Prediction"<<std::endl;

   double std_x = 2;
   double std_y = 2;
   double std_theta = 0.05; 

   
   std::default_random_engine gen;
   normal_distribution<double> dist_x(0.0 , std_x);
   normal_distribution<double> dist_y(0.0, std_y);
   normal_distribution<double> dist_theta(0.0, std_theta);   

   int i=0;

   while(i<num_particles){
     
    if(fabs(yaw_rate) > 0){

      double const1 = velocity/yaw_rate;
      double const2 = yaw_rate * delta_t;
      particles[i].x = particles[i].x + const1*(sin(particles[i].theta + const2) - sin(particles[i].theta));
      particles[i].y = particles[i].y + const1*(cos(particles[i].theta) - cos(particles[i].theta + const2));
      particles[i].theta = particles[i].theta + const2;  
       
    } 
    else{
        particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
        particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta); 
    }
    
   
    particles[i].x +=  dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);

   i=i+1;
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

    unsigned int i=0;
    
    while ( i < observations.size()) {
    
    // initial distance to nearest prediction and its id
    int nearest_prediction_id = -1;
    double distance = 100000000;
    unsigned int j=0;

    
    	while ( j < predicted.size()) {
      			  double current_dist = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      
     			  if (current_dist < distance) {
    				    distance = current_dist;
        			    nearest_prediction_id = predicted[j].id;
      			  }
              j=j+1;               
         }			
    
       observations[i].id = nearest_prediction_id;
       i=i+1;
   }


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
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


  std::cout<<" Update weights "<<std::endl;
  for (int i = 0; i < num_particles; i++) { 
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;

    unsigned int j=0;

    vector<LandmarkObs> t_observations; 

    std::cout<<" While observations "<<std::endl;
    while (j < observations.size()) { 
      double mapx;
      mapx = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
      
      double mapy;
      mapy = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);

      LandmarkObs observation;
      observation.x = mapx;
      observation.y = mapy;
      t_observations.push_back(observation);

      j=j+1;
    }

    unsigned int k=0;

    vector<LandmarkObs> predictions;
    std::cout<<" While landmark "<<std::endl;
    while (k < map_landmarks.landmark_list.size()) {
     
      if (dist(map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f, x, y) <= sensor_range) {
        predictions.push_back(LandmarkObs{ map_landmarks.landmark_list[k].id_i, map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f });
      }

      k=k+1;
    }

    dataAssociation(predictions, t_observations);

    unsigned int l=0; 

    double weight = 1.0;
    std::cout<<" While t observation "<<std::endl;
    while (l< t_observations.size()) {
      double  pred_x, pred_y;
       unsigned int m=0;
       while (m < predictions.size()) {
        if (predictions[m].id == t_observations[l].id) {
          pred_x = predictions[m].x;
          pred_y = predictions[m].y;

          
        }
        m=m+1;
       }

      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];

      
      double gauss_norm;
      gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

      double exponent;
      exponent = (pow(t_observations[l].x - pred_x, 2) / (2 * std_landmark[0]*std_landmark[0]))
                 + (pow(t_observations[l].y - pred_y, 2) / (2 * std_landmark[1]*std_landmark[1]));

      double w;
      w = gauss_norm * exp(-exponent);

      weight *= w;
           
      l=l+1;
       std::cout<<" end While l "<<std::endl; 
    }
   
    
    particles[i].weight = weight;

    std::cout<<" end update "<<std::endl;
  }



  

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

    std::cout<<" Resample "<<std::endl;
    vector<double> weights;
    double max = 0;
    std::default_random_engine gen;


    int i=0;
    while(i < num_particles) {
      weights.push_back(particles[i].weight);
      if ( particles[i].weight > max ) {
        max = particles[i].weight;
      }
      i=i+1;
    }

    
    uniform_real_distribution<double> dist_double(0, 2*max);
    uniform_int_distribution<int> dist_index(0, num_particles - 1);

    
    int index = dist_index(gen);

    double beta = 0.0;

    
    vector<Particle> resampled;
    i=0;
   
    while( i < num_particles) {
      beta += dist_double(gen) * 2.0;
      while( beta > weights[index]) {
        beta -= weights[index];
        index = (index + 1) % num_particles;
      }
      resampled.push_back(particles[index]);
      i=i+1;
    }

    particles = resampled;

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
