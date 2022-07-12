//
// Created by yyh on 22-7-12.
//

#ifndef CENTRAL_VOTING_SMARTDOWNSAMPLE_H
#define CENTRAL_VOTING_SMARTDOWNSAMPLE_H
#include "Eigen/Core"
class SmartDownSample {
 public:
  SmartDownSample();

  void setLeafSize(const Eigen::Vector4f &leaf_size);
 private:
  Eigen::Vector4f leaf_size;



};

#endif  // CENTRAL_VOTING_SMARTDOWNSAMPLE_H
