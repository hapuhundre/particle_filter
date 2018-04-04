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
	// 1. 设置粒子个数与其初始位置（根据GPS数据的x,y,theta和不确定参数估计得来），所有
	//    权重均设置为1
	// 2. 对每个粒子添加高斯噪声

	
	
	// 粒子个数，需调参
	num_particles = 10;
	
	// 正态分布的估计值
	normal_distribution<double> norm_x(x, std[0]);
	normal_distribution<double> norm_y(y, std[1]);
	normal_distribution<double> norm_theta(theta, std[2]);
	
	default_random_engine random_gen;

	for(int i=0; i<num_particles; ++i){
        Particle p;
		p.id = i;
		// p.x = x;
		// p.y = y;
		// p.theta = theta;
		
		// 添加噪声
		p.x = norm_x(random_gen);
		p.y = norm_y(random_gen);
		p.theta = norm_theta(random_gen);
		
		p.weight = 1.0; // all weights to 1

		particles.push_back(p);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// 对每个粒子添加测量值与高斯噪声
	// 添加噪声可使用std::normal_distribution & std::default_random_engine.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	normal_distribution<double> norm_x(0, std_pos[0]);
	normal_distribution<double> norm_y(0, std_pos[1]);
	normal_distribution<double> norm_theta(0, std_pos[2]);
	
    default_random_engine random_gen;

	for(int i=0; i<num_particles; ++i){
		if(fabs(yaw_rate)<0.000001){
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}else{
            particles[i].x += (velocity/yaw_rate) * ( sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta) );
            particles[i].y += (velocity/yaw_rate) * ( cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t) );
            particles[i].theta += yaw_rate*delta_t;
		}
	    // 添加噪声
		particles[i].x += norm_x(random_gen);
		particles[i].y += norm_y(random_gen);
		particles[i].theta += norm_theta(random_gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// 遍历预测值与观察值，选出最接近的id对(assign the observed measurement to this particular landmark.)
	// dist函数参见helper_functions.h 
    for(int i=0; i < observations.size(); ++i){
		double min_dist = 1E6;
		for(int j=0; j < predicted.size(); ++j){
            double temp_dist = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
            if(temp_dist < min_dist){
				min_dist = temp_dist;
				observations[i].id = predicted[j].id;
			}
		}
	}
}

double ParticleFilter::BivariateCaseGaussianPdf(double x, double y,
                                                double mean_x, double mean_y,
												double std_x, double std_y, double rho){
	// 公式为：https://wikimedia.org/api/rest_v1/media/math/render/svg/c6fc534bfde62d6d2b3b743b0c3fa2fb7fc3174a
    if(rho>=1 || rho<=-1) rho=0.0;
	double z1 = 2 * M_PI * std_x * std_y * sqrt(1-rho*rho);
	double z2 = -1 / (2*(1-rho*rho));
	double z3 = (x-mean_x)*(x-mean_x) / (std_x*std_x);
	double z4 = (y-mean_y)*(y-mean_y) / (std_y*std_y);
	double z5 = 2*rho*(x-mean_x)*(y-mean_y) / (std_x*std_y);
	return exp(z2*(z3+z4-z5)) / z1;
}



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// 多变量高斯分布更新权重 more to see: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// 观测值是基于汽车坐标系的，粒子点则是MAP坐标系，两者需要坐标转换（包括旋转和平移，但不包括放缩）
	// 2d坐标转换ref:
	// 1. https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	// 2. http://planning.cs.uiuc.edu/node99.html (equation 3.33)
    double all_weights = 0.0;

    weights.clear();
    for(int i=0; i<num_particles; ++i){
        // Step 1 将地图中的观测点与粒子点进行距离测量，小于sensor_range的map_landmarks点添加到prediction中
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;
        
		vector<LandmarkObs> pred;
		double map_x, map_y;
		int map_id;
        for(int j=0; j<map_landmarks.landmark_list.size(); ++j){
			double map_x = map_landmarks.landmark_list[j].x_f;
			double map_y = map_landmarks.landmark_list[j].y_f;
			int map_id = map_landmarks.landmark_list[j].id_i;
			double mp_dist = dist(p_x, p_y, map_x, map_y);
			if(mp_dist < sensor_range)
				pred.push_back( LandmarkObs{map_id, map_x, map_y} );
		}

	    // Step 2 转换observations的坐标系
		vector<LandmarkObs> trans_obser;
        for(int j=0; j<observations.size(); ++j){
			double trans_x = observations[j].x * cos(p_theta) - observations[j].y * sin(p_theta) + p_x;
			double trans_y = observations[j].x * sin(p_theta) + observations[j].y * cos(p_theta) + p_y;
            trans_obser.push_back( LandmarkObs{ observations[j].id, trans_x, trans_y } );
		}

	    // Step 3 dataAssociation
        dataAssociation(pred, trans_obser);

		// Step 4 计算先验概率
        // 找到每一个全局观察点与预测id相同时
		// prob 累乘其距离的正太分布

		double prob = 1.0;
		for(int j=0; j<trans_obser.size(); ++j){
            double global_x = trans_obser[j].x;
			double global_y = trans_obser[j].y;
			int global_id = trans_obser[j].id;
			
			for(int k=0; k<pred.size(); ++k){
				if(pred[k].id == global_id){
                    prob *= BivariateCaseGaussianPdf(global_x, global_y, pred[k].x, pred[k].y,
					                                 std_landmark[0], std_landmark[1], 0);
					break;
				}
			}
		}
		particles[i].weight = prob;
        all_weights += prob;
	}
	
	// 正则化
    for(int i=0; i<num_particles; ++i){
		particles[i].weight /= all_weights;
		weights.push_back(particles[i].weight);
	}
}

void ParticleFilter::resample() {
	// 根据粒子权重占比对其进行重采样
	// NOTE:
	// std::random_device
	// http://en.cppreference.com/w/cpp/numeric/random/random_device
	// std::discrete_distribution
	// http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
    vector<Particle> new_p;

	// 生成随机数
	random_device rd;
    default_random_engine random_gen(rd());
    
	for(int i=0;i<num_particles;++i){
		discrete_distribution<int> idx(weights.begin(), weights.end());
		// new_p[i] = particles[idx(random_gen)]
		new_p.push_back(particles[idx(random_gen)]);
	}
	particles = new_p;
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
