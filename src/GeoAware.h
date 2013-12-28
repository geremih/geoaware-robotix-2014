#ifndef GEOAWARE_H
#define GEOAWARE_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <map>

#define CIRCLE 1
#define TRIANGLE 3
#define SQUARE 4
#define PENTAGON 5
#define HEXAGON 6

#define PRIMARY_THRESHOLD 69
#define BLACK_THRESHOLD 65
#define EPSILON 10

std::map<int,std::string> vtxToShape;

#endif
