#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

// constants
unsigned int NUM_ITERS = 150;
unsigned int NUM_FEATURE_POINTS = 4;

// function definitions
std::vector<std::vector<float>> parse();
void init_start_end_points(std::vector<std::vector<float>> &start_points, std::vector<std::vector<float>> &end_points);
void init_task(vpServo &task, std::vector<vpFeaturePoint> &p, std::vector<vpFeaturePoint> &pd, const std::vector<std::vector<float>> &start_points, const std::vector<std::vector<float>> &end_points);
void visual_servo_loop(const std::vector<std::vector<float>> &results, vpServo &task, std::vector<vpFeaturePoint> &p);