#ifndef OBJDETECTOR_H
#define OBJDETECTOR_H


#include "ros/ros.h"
#include <ros/package.h>

#include "class_detector.h"
#include <string>

using namespace std;

class ObjDetector {

private:
    string pkg_path;
    string cfg_path;
    string weights_path;

    Detector detector;
    Config config;

public:
    ObjDetector(const string & config, const string & cfg, const string & weights);
    vector<Result> run_detector();
}

#endif