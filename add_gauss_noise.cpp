//
// Created by yyh on 22-8-11.
//

#include "add_gauss_noise.h"

/**
* @brief   : 设置输入点云
* @param[I]: cloud_in (输入点云)
* @param[O]: none
* @return  : none
* @note    :
**/
void AddGaussNoise::setInputCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in)
{
  m_cloud_in = cloud_in;
  is_setInputCloud = true;
}

/**
* @brief   : 设置高斯噪声参数
* @param[I]: mu (均值，默认0)
* @param[I]: sigma (标准差，默认1)
* @param[O]: none
* @return  : none
* @note    :
**/
void AddGaussNoise::setParameters(double mu, double sigma)
{
  if (sigma > 0)
  {
    m_mu = mu;
    m_sigma = sigma;
    is_setParameters = true;
  }
  else
  {
    PCL_ERROR("->sigma应大于0！\a\n");
    system("pause");
    abort();
  }
}

/**
* @brief   : 执行添加高斯噪声
* @param[I]: cloud_out (添加高斯噪声后的点云)
* @param[O]: none
* @return  : none
* @note    :
**/
void AddGaussNoise::addGaussNoise(pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
  boost::mt19937 zgy;								//等分布均匀伪随机数发生器
  zgy.seed(static_cast<unsigned int>(time(nullptr)));	//随机种子
  boost::normal_distribution<> nd(m_mu, m_sigma);	//定义正态分布，均值为mu，标准差为sigma
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> gauss_noise(zgy, nd);	//生成高斯噪声

  pcl::PointCloud<pcl::PointXYZ> cloud_gauss;	//声明高斯噪声点云
  cloud_gauss = m_cloud_in;					//将原始点云拷贝给高斯噪声点云，用于下面的平移

  for (size_t i = 0; i < cloud_gauss.size(); i++)
  {
    cloud_gauss.points[i].x += static_cast<float> (gauss_noise());
    cloud_gauss.points[i].y += static_cast<float> (gauss_noise());
    cloud_gauss.points[i].z += static_cast<float> (gauss_noise());
  }
  std::cout<<"gauss size: "<<cloud_gauss.points.size()<<" scene size: "<<m_cloud_in.points.size()<<std::endl;
  cloud_out = m_cloud_in + cloud_gauss;	//将原始点云与噪声点云合并，得到添加高斯噪声后的点云
}
