#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>

std::vector<std::vector<float>> parse_velocities(std::string path)
{
    std::ifstream file(path);
    std::string str; 
    std::vector<std::vector<float>> results;

    // ignore the first 10 lines in the file
    for (unsigned int i = 0; i < 10; ++i)
    {
        std::getline(file, str);
    }

    // parsing loop
    while (std::getline(file, str))
    {
        // the first getline ignored the iter line
        // now ignore the next 4 lines that contain the feature points
        for (unsigned int i = 0; i < 4; ++i)
        {
            std::getline(file, str);
        }

        // now we have the velocity vector we want in the form "v: vx vy vz wx wy wz"
        std::getline(file, str);
        // remove the leading "v: "
        str.erase(0,3);

        // now parse the space delimited string to obtain the velocities
        std::vector<float> result;
        std::stringstream s_stream(str);
        while(s_stream.good()) {
            std::string substr;
            std::getline(s_stream, substr, ' '); //get first string delimited by space
            result.push_back(stof(substr));
        }
        results.push_back(result);
    }

    // // print results for sanity check
    // for (unsigned int i = 0; i < results.size(); ++i)
    // {
    //     for (unsigned int j = 0; j < results[i].size(); ++j)
    //     {
    //         std::cout << results[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    return results;
}

/*
    Parse the desired_output.txt and visual_servo_output.txt and compare the outputted velocity results
*/
int main()
{
    // parse the velocity results
    std::vector<std::vector<float>> vs_output = parse_velocities("datafiles/visual_servo_output.txt");
    std::vector<std::vector<float>> des_output = parse_velocities("datafiles/desired_output.txt");

    // calculate the percent error between the visual servo output and desired output velocities
    std::vector<float> errors;
    float total_error = 0;
    unsigned int num_points = vs_output.size();
    unsigned int vel_size = vs_output[0].size();

    for (unsigned int i = 0; i < num_points; ++i)
    {
        float error = 0;
        for (unsigned int j = 0; j < vel_size; ++j)
        {
            float e = des_output[i][j];
            float a = vs_output[i][j];
            error += fabs(100 * fabs(e-a)/a);
        }
        errors.push_back(error/vel_size);
        total_error += (error/vel_size);
    }

    // print the average error
    std::cout << "avg error: " << total_error/num_points << "%" << std::endl;

    return 0;
}