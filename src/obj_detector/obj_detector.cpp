#ifndef OBJDETECTOR_CPP
#define OBJDETECTOR_CPP

#include "obj_detector.h"

/* Constructor for object detector
 *  package - name of the package for locating config folder 
 *  cfg     - name of the model config file
 *  weights - name of the weights file
 */
ObjDetector::OBjDetector(const string & package, const string & cfg, const string & weights) {
    // setup the required paths to model and weights
    pkg_path = ros::package::getpath(package);
    cfg_path = pkg_path + "/src/config/" + cfg;
    weights_path = pkg_path + "/src/config/" + weights;

    // configure and initialise the detector
    config.file_model_cfg = cfg_path;
    config.file_model_weights = weights_path;
    config.inference_precison = FP16; // INT8, FP16, FP32
    config.detect_thresh = 0.5;
    detector.init(config);
}

/* Perform inferene using the object detector
 *  image   - the image to perform inference on 
 */
vector<result> ObjDetector::RunDetector(cv::Mat image) const {
    vector<result> res;

    detector.detect(image, res);

    return res;
}

#endif