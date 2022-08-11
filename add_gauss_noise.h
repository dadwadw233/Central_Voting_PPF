//
// Created by yyh on 22-8-11.
//

#ifndef CENTRAL_VOTING_ADD_GAUSS_NOISE_H
#define CENTRAL_VOTING_ADD_GAUSS_NOISE_H

#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/random.hpp>		//随机数所需头文件

class AddGaussNoise
{
 public:

  /**
	* @brief   : 设置输入点云
	* @param[I]: cloud_in (输入点云)
	* @param[O]: none
	* @return  : none
	* @note    :
	**/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in);

  /**
	* @brief   : 设置高斯噪声参数
	* @param[I]: mu (均值，默认0)
	* @param[I]: sigma (标准差，默认1)
	* @param[O]: none
	* @return  : none
	* @note    :
	**/
  void setParameters(double mu = 0.0, double sigma = 1.0);

  /**
	* @brief   : 执行添加高斯噪声
	* @param[I]: cloud_out (添加高斯噪声后的点云)
	* @param[O]: none
	* @return  : none
	* @note    :
	**/
  void addGaussNoise(pcl::PointCloud<pcl::PointXYZ> &cloud_out);
 private:

  pcl::PointCloud<pcl::PointXYZ> m_cloud_in;		//输入点云
  bool is_setInputCloud = false;					//是否设置输入点云
  double m_mu, m_sigma;							//高斯分布参数
  bool is_setParameters = false;					//是否设置高斯分布参数
};


#endif  // CENTRAL_VOTING_ADD_GAUSS_NOISE_H
