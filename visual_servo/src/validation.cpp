#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <numeric>
#include <stdlib.h>
#include <math.h>

/*
    Parse the velocity vectors
*/
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
    Calculate percent difference
*/
std::vector<float> compute_error_vector(const std::vector<std::vector<float>> &vs_output, const std::vector<std::vector<float>> &des_output)
{
    // init parameters
    std::vector<float> errors;
    unsigned int num_points = vs_output.size();
    unsigned int vel_size = vs_output[0].size();

    // calculate percent difference
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
    }

    return errors;
}

/*
    Compute average error and standard deviation of errors
*/ 
void compute_metrics(float &avg_error, float &std_dev, const std::vector<float> &errors)
{
    float sum = std::accumulate(errors.begin(), errors.end(), 0.0);
    avg_error = sum / errors.size();

    float sq_sum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
    std_dev = std::sqrt(sq_sum / errors.size() - avg_error * avg_error);
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
    std::vector<float> errors = compute_error_vector(vs_output, des_output);

    // compute metrics
    float avg_error = 0;
    float std_dev = 0;
    compute_metrics(avg_error, std_dev, errors);
    std::cout << "avg error: " << avg_error << "%" << std::endl;
    std::cout << "std_dev: " << std_dev << "%" << std::endl;

    return 0;
}