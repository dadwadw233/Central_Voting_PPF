//
// Created by yyh on 22-7-20.
//
#include "PPFEstimation.h"


void PPFEstimation::setDiscretizationSteps(
    const float &angle_discretization_step,
    const float &distance_discretization_step) {
  this->angle_discretization_step = angle_discretization_step;
  this->distance_discretization_step = distance_discretization_step;
}
PPFEstimation::PPFEstimation() {}
