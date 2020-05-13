#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>

/*
    Parse txt file to obtain feature points
*/
int main()
{
    std::ifstream file("data_stripped.txt");
    std::string str; 
    std::vector<std::vector<float>> results;

    // parsing loop
    while (std::getline(file, str))
    {
        // the first getline ignores the iter line
        // now parse the next 4 lines to get the 4 feature points
        for (unsigned int j = 0; j < 4; ++j)
        {
            std::getline(file, str);
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

    // print results for sanity check
    for (unsigned int i = 0; i < results.size(); ++i)
    {
        for (unsigned int j = 0; j < results[i].size(); ++j)
        {
            std::cout << results[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}