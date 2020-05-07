/*! \example tutorial-ibvs-4pts.cpp */
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

std::vector<std::vector<float>> parse()
{
    std::ifstream file("test2.txt");
    std::string str; 
    std::vector<std::vector<float>> results;

    while (std::getline(file, str))
    {
        for (unsigned int j = 0; j < 4; ++j)
        {
            std::getline(file, str); // iter
            std::vector<float> result;
            std::stringstream s_stream(str);
            while(s_stream.good()) {
                std::string substr;
                std::getline(s_stream, substr, ','); //get first string delimited by comma
                result.push_back(stof(substr));
            }
            results.push_back(result);
        }
    }

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

int main()
{
    // parse txt file to get data points
    std::vector<std::vector<float>> results = parse();

    std::cout << results.size() << std::endl;

    // instantiate the visual servo task
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA); // eye in hand visual servo
    task.setInteractionMatrixType(vpServo::CURRENT); // compute interaction matrix from current visual features
    task.setLambda(0.5); // gain

    // define four visual features as 2D points in image-plane
    vpFeaturePoint p[4]; // current point feature
    vpFeaturePoint pd[4]; // desired point feature

    // set current and desired points
    p[0].set_xyZ(0.1693, -0.244827, 0.969685);
    p[1].set_xyZ(0.286181, -0.0874326, 1.01422);
    p[2].set_xyZ(0.131836, 0.036305, 1.03031);
    p[3].set_xyZ(0.00989196, -0.11293, 0.985785);
    pd[0].set_xyZ(-0.133333, -0.133333, 0.75);
    pd[1].set_xyZ(0.133333, -0.133333, 0.75);
    pd[2].set_xyZ(0.133333, 0.133333, 0.75);
    pd[3].set_xyZ(-0.133333, 0.133333, 0.75);

    for (unsigned int i = 0; i < 4; i++)
    {
        task.addFeature(p[i], pd[i]); // add curent and desired feature to visual servo task
    }

    std::cout << "current features points" << std::endl;
    p[0].print();
    p[1].print();
    p[2].print();
    p[3].print();
    std::cout << "desired feature points" << std::endl;
    pd[0].print();
    pd[1].print();
    pd[2].print();
    pd[3].print();

    // visual servo loop
    for (unsigned int iter = 0; iter < 150; iter++)
    {
        std::cout << "iter: " << iter << std::endl;
        for (unsigned int i = 0; i < 4; ++i)
        {
			// update current visual features
            float x = results[iter*4 + i][1];
            float y = results[iter*4 + i][2];
            float Z = results[iter*4 + i][0];
            p[i].set_xyZ(x, y, Z);
        }
        vpColVector v = task.computeControlLaw();

        p[0].print();
        p[1].print();
        p[2].print();
        p[3].print();
        printf("v: %f %f %f %f %f %f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
    }

    return 0;
}
