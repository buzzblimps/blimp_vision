#pragma once

#include <ctime>
#include <vector>
#include <iostream>
#include <cstring>

class Benchmarker{
    public:
    std::vector<clock_t> times;
    std::vector<std::string> flags;

    void benchmarkFirst(std::string flag){
        times.clear();
        flags.clear();
        times.push_back(clock());
        flags.push_back(flag);
    }

    void benchmark(std::string flag){
        times.push_back(clock());
        flags.push_back(flag);
    }

    void benchmarkPrint(){
        if(times.size() < 2) return;
        double deltaTotal = double(times[times.size()-1] - times[0]) / CLOCKS_PER_SEC;
        for(unsigned int i=1; i<times.size(); i++){
            double deltaTime = double(times[i] - times[i-1]) / CLOCKS_PER_SEC;
            float percentTime = deltaTime / deltaTotal * 100;
            percentTime = int(percentTime * 100) / 100.0;
            std::string percentString = std::to_string(percentTime);
            if(percentString.at(1) == '.') percentString = " " + percentString;
            percentString = percentString.substr(0,5);

            std::cout << percentString << "%, " << flags[i-1] << " -> " << flags[i] << ": " << deltaTime << std::endl;
        }
        std::cout << "Total: " << 1.0/deltaTotal << "Hz" << std::endl;
    }
};