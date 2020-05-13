#include <visual_servo/visual_servo.h>

/*
    Parse the stripped data file that contains the feature point info
    Eventually this won't be needed, instead we'll get feature point info from tracker
*/
std::vector<std::vector<float>> parse()
{
    // read in the data file
    std::ifstream file("datafiles/data_stripped.txt");
    std::string str; 
    std::vector<std::vector<float>> results;

    // parse the feature points
    while (std::getline(file, str))
    {
        // the first getline ignores the iter line
        // now parse the next 4 lines to get the 4 feature points
        for (unsigned int j = 0; j < NUM_FEATURE_POINTS; ++j)
        {
            std::getline(file, str);
            std::vector<float> result;
            std::stringstream s_stream(str);
            while(s_stream.good()) {
                std::string substr;
                std::getline(s_stream, substr, ','); // get first string delimited by comma
                result.push_back(stof(substr));
            }
            results.push_back(result);
        }
    }

    // // print out the results from parsing
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
    Get the initial and final feature points for the visual servo task
    Currently uses hard-coded values obtained from tutorial-ibvs-4pts.cpp
    Eventually will need to be obtained/calculated from tracker
*/
void init_start_end_points(std::vector<std::vector<float>> &start_points, std::vector<std::vector<float>> &end_points)
{
    // xyZ format
    start_points.push_back({0.1693, -0.244827, 0.969685});
    start_points.push_back({0.286181, -0.0874326, 1.01422});
    start_points.push_back({0.131836, 0.036305, 1.03031});
    start_points.push_back({0.00989196, -0.11293, 0.985785});
    end_points.push_back({-0.133333, -0.133333, 0.75});
    end_points.push_back({0.133333, -0.133333, 0.75});
    end_points.push_back({0.133333, 0.133333, 0.75});
    end_points.push_back({-0.133333, 0.133333, 0.75});
}

/*
    Initializes the visual servoing task
*/
void init_task(vpServo &task, std::vector<vpFeaturePoint> &p, std::vector<vpFeaturePoint> &pd, const std::vector<std::vector<float>> &start_points, const std::vector<std::vector<float>> &end_points)
{
    // set up the task
    task.setServo(vpServo::EYEINHAND_CAMERA); // eye in hand visual servo
    task.setInteractionMatrixType(vpServo::CURRENT); // compute interaction matrix from current visual features
    task.setLambda(0.5); // gain

    // set current and desired points
    for (unsigned int i = 0; i < NUM_FEATURE_POINTS; ++i)
    {
        float x_start = start_points[i][0];
        float y_start = start_points[i][1];
        float Z_start = start_points[i][2];
        p[i].set_xyZ(x_start, y_start, Z_start);

        float x_end = end_points[i][0];
        float y_end = end_points[i][1];
        float Z_end = end_points[i][2];
        pd[i].set_xyZ(x_end, y_end, Z_end);
    }

    for (unsigned int i = 0; i < NUM_FEATURE_POINTS; i++)
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
}

/*
    Visual servo loop
    Currently gets the feature points from the results vector
    Eventually will need to get from tracker at each timestamp
*/
void visual_servo_loop(const std::vector<std::vector<float>> &results, vpServo &task, std::vector<vpFeaturePoint> &p)
{
    // visual servo loop
    for (unsigned int iter = 0; iter < NUM_ITERS; iter++)
    {
        std::cout << "iter: " << iter << std::endl;\

        for (unsigned int i = 0; i < NUM_FEATURE_POINTS; ++i)
        {
			// update current visual features
            float x = results[iter*NUM_FEATURE_POINTS + i][1];
            float y = results[iter*NUM_FEATURE_POINTS + i][2];
            float Z = results[iter*NUM_FEATURE_POINTS + i][0];
            p[i].set_xyZ(x, y, Z);
        }
        vpColVector v = task.computeControlLaw();

        p[0].print();
        p[1].print();
        p[2].print();
        p[3].print();
        printf("v: %f %f %f %f %f %f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
    }
}

/*
    Uses parsed data points to perform visual servo task
    Used to create visual_servo_output.txt by redirecting output to the txt file
*/
int main()
{
    // parse txt file to get data points
    std::vector<std::vector<float>> results = parse();

    // instantiate the visual servo task
    vpServo task;

    // define four visual features as 2D points in image-plane
    std::vector<vpFeaturePoint> p(NUM_FEATURE_POINTS); // current point feature
    std::vector<vpFeaturePoint> pd(NUM_FEATURE_POINTS); // desired point feature

    // get the start and end points for the task
    std::vector<std::vector<float>> start_points;
    std::vector<std::vector<float>> end_points;
    init_start_end_points(start_points, end_points);

    // initialize the task
    init_task(task, p, pd, start_points, end_points);

    // visual servo loop
    visual_servo_loop(results, task, p);

    return 0;
}
