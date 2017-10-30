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
	
	default_random_engine gen;

	num_particles=1;
	// std::vector<Particle> particles[num_particles];

	Particle inicial;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);


	// std::vector<int> associations;
	// std::vector<double> sense_x;
	// std::vector<double> sense_y;

	for(int i=0;i<num_particles;i++)
	{
		inicial.id=i;
		inicial.x=dist_x(gen);
		inicial.y=dist_y(gen);
		inicial.theta=dist_theta(gen);
		inicial.weight=1.0;
		// inicial.associations[];
		// inicial.sense_x[];
		// inicial.sense_y[];

		particles.push_back(inicial);

		// cout<<particles[i].id<<endl;
		// cout<<particles[i].x<<endl;

	}
	
	is_initialized=true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);


	for(int i=0;i<num_particles;i++)
	{

		// dist_x(gen);
		// dist_y(gen);
		// dist_theta(gen);

		if(fabs(yaw_rate) < 0.0001)
		{
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
	      	particles[i].y += velocity * delta_t * sin(particles[i].theta);
	      	particles[i].theta+=yaw_rate*delta_t;
	    }

    	else
    	{
    		particles[i].x+=(velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t)-sin(particles[i].theta));
			particles[i].y+=(velocity/yaw_rate)*(cos(particles[i].theta)-cos(particles[i].theta+yaw_rate*delta_t));
			particles[i].theta+=yaw_rate*delta_t;
		}

	particles[i].x+=dist_x(gen);
	particles[i].y+=dist_y(gen);
	particles[i].theta+=dist_theta(gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.



	if(observations.size()>0)
	{
		// cout<<observations.size()<<endl;
		// cout<<predicted.size()<<endl;
		// cout<<observations[0].x<<endl;
		// cout<<observations[0].y<<endl;
		// cout<<observations[0].id<<endl;

		

		for(unsigned int i=0;i<observations.size();i++)
		{

		double distancia_min=9999;
		int index=-1;

			for(unsigned int j=0;j<predicted.size();j++)
			{
				if(dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y)<distancia_min)
				{
					distancia_min=dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
					index=predicted[j].id;
				}
			}

		observations[i].id=index;
		// cout<<observations[i].id<<endl;
			
		}

	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
	
	

	// cout<<"mapa "<<endl;
	// cout<<map_landmarks.landmark_list.size()<<endl;


	for(int i=0;i<particles.size();i++)
	{

		//check which landmarks are within the sensor range
		std::vector<LandmarkObs> within_range;
		LandmarkObs to_include;

		for(int j=0;j<map_landmarks.landmark_list.size();j++)
		{
			if(dist(particles[i].x,particles[i].y,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f)<=sensor_range)
			{
				to_include.id=map_landmarks.landmark_list[j].id_i;
				to_include.x=map_landmarks.landmark_list[j].x_f;
				to_include.y=map_landmarks.landmark_list[j].y_f;
				within_range.push_back(to_include);
			}
		
		}
	
	//convert coordinates -> car 2 map
 	vector<LandmarkObs> converted;
 	// double conv_x;
 	// double conv_y;

    for (int k = 0; k < observations.size(); k++) {
      to_include.x = cos(particles[i].theta)*observations[k].x - sin(particles[i].theta)*observations[k].y + particles[i].x;
      to_include.y = sin(particles[i].theta)*observations[k].x + cos(particles[i].theta)*observations[k].y + particles[i].y;
      to_include.id=observations[k].id;

      converted.push_back(to_include);
    }

	// cout<<"within "<<endl;
	// cout<<within_range.size()<<endl;

	dataAssociation(within_range,converted);

	


  	

  	// particles[i].weight = 1.0;


	for(int l=0;l<converted.size();l++)
	{

		for(int m=0;m<within_range.size();m++)
		{

			if(within_range[m].id==converted[l].id)
			{

				particles[i].weight*=(1/(2*M_PI*std_landmark[0]*std_landmark[1]))*exp(-(pow(within_range[m].x-converted[l].x,2)/(2*pow(std_landmark[0],2))+(pow(within_range[m].y-converted[l].y,2)/(2*pow(std_landmark[1],2)))));
			}
		}

	}




	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


 // vector<Particle> new_particles;
 // default_random_engine gen;

 //  // get all of the current weights
 //  vector<double> weights;
 //  for (int i = 0; i < num_particles; i++) {
 //    weights.push_back(particles[i].weight);
 //  }

 //  // generate random starting index for resampling wheel
 //  uniform_int_distribution<int> uniintdist(0, num_particles-1);
 //  auto index = uniintdist(gen);

 //  // get max weight
 //  double max_weight = *max_element(weights.begin(), weights.end());

 //  // uniform random distribution [0.0, max_weight)
 //  uniform_real_distribution<double> unirealdist(0.0, max_weight);

 //  double beta = 0.0;

 //  // spin the resample wheel!
 //  for (int i = 0; i < num_particles; i++) {
 //    beta += unirealdist(gen) * 2.0;
 //    while (beta > weights[index]) {
 //      beta -= weights[index];
 //      index = (index + 1) % num_particles;
 //    }
 //    new_particles.push_back(particles[index]);
 //  }

 //  particles = new_particles;


	std::default_random_engine gen;
    std::vector<Particle> resampled_particles;
    resampled_particles.resize(num_particles);
    //resampled_particles = std::vector<Particle>(num_particles);
    std::discrete_distribution<> d(weights.begin(), weights.end());
    //std::map<int, int> m;
    for(int i=0; i<num_particles; i++) {
        resampled_particles[i] = particles[d(gen)];
    }
    particles=resampled_particles;


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
