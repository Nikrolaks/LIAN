#include "config.h"

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"


float Config::getParamValue(std::size_t i) const {
    return searchParams_[i];
}

const std::string& Config::getMapFileName() const {
    return mapFileName_;
}

Config::Config(const std::string& fileName) {
    std::string value;
    float angle;
    int distance;
    float weight;
    unsigned int steplimit;
    float loglevel;
    float curvatureHeuriscitWeight;
    double pivotRadius = 0;
    float decreaseDistance;
    int distanceMin;
    int numOfParentsToIncreaseRadius;
    bool postsmoother;
    std::stringstream stream;

    TiXmlDocument doc(fileName.c_str());
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file.");
    }

    TiXmlElement *root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        throw std::runtime_error((std::stringstream() << "No '" << CNS_TAG_ROOT << "' element found in XML file.").str());
    }

    TiXmlElement *algorithm = root->FirstChildElement(CNS_TAG_ALGORITHM);
    if (!algorithm) {
        throw std::runtime_error((std::stringstream() << "No '" << CNS_TAG_ALGORITHM << "' element found in XML file.").str());
    }

    TiXmlElement *element;

    element = getElement(root, CNS_TAG_MAP);
    mapFileName_ = serialize<std::string>(element);

        searchParams_ = std::vector<float>(CN_PT_NUM);

        element = getElement(algorithm, CNS_TAG_ANGLELIMIT, CNS_TAG_ALGORITHM);
        angle = serialize<float>(element);
        if (angle < 0)
            angle *= -1;
        if (angle > 180) {
            std::cout << "Warning! Trying to set angle limit to more than 180. " <<
                             "Angle limit is set to 180 instead." << std::endl;
            angle = 180;
        }
        searchParams_[CN_PT_AL] = angle;


        element = getElement(algorithm, CNS_TAG_DISTANCE, CNS_TAG_ALGORITHM);
        distance = serialize<float>(element);
        if (distance < 0) {
            distance *= -1;
        }

        searchParams_[CN_PT_D] = distance;


        element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_WEIGHT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_W << "." << std::endl;
            weight = CN_PTD_W;
        } else {
            weight = serialize<float>(element);
        }
        searchParams_[CN_PT_W] = weight;

        element = algorithm->FirstChildElement(CNS_TAG_STEPLIMIT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_STEPLIMIT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            steplimit = 0;
        } else {
            steplimit = serialize<float>(element);
        }
        searchParams_[CN_PT_SL] = steplimit;

        element = algorithm->FirstChildElement(CNS_TAG_CURVHEURWEIGHT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_CURVHEURWEIGHT << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            curvatureHeuriscitWeight = 0;
        } else {
            curvatureHeuriscitWeight = serialize<float>(element);
        }
        searchParams_[CN_PT_CHW] = curvatureHeuriscitWeight;

        element = algorithm->FirstChildElement(CNS_TAG_SMOOTHER);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_SMOOTHER << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: false." << std::endl;
            postsmoother = 0;
        } else {
            postsmoother = serialize<float>(element);
        }
        searchParams_[CN_PT_PS] = postsmoother;

        element = algorithm->FirstChildElement(CNS_TAG_DECRDISTFACTOR);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_DECRDISTFACTOR << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_DDF << "." << std::endl;
            decreaseDistance = CN_PTD_DDF;
        } else {
            decreaseDistance = serialize<float>(element);
        }
        searchParams_[CN_PT_DDF] = decreaseDistance;


        element = algorithm->FirstChildElement(CNS_TAG_DISTANCEMIN);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_DISTANCEMIN << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_DMIN << "." << std::endl;
            distanceMin = CN_PTD_DMIN;
        } else {
            distanceMin = serialize<float>(element);
        }
        searchParams_[CN_PT_DM] = distanceMin;


        element = algorithm->FirstChildElement(CNS_TAG_PIVOTCIRCLE);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_PIVOTCIRCLE << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: 0." << std::endl;
            pivotRadius = 0;
        } else {
            pivotRadius = serialize<float>(element);
        }
        searchParams_[CN_PT_PC] = pivotRadius;

        element = algorithm->FirstChildElement(CNS_TAG_NOFPTOINCRAD);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_NOFPTOINCRAD << "' element found inside '" << CNS_TAG_ALGORITHM
                      << "' section. Set to default value: " << CN_PTD_NOFPTOINCRAD << "." << std::endl;
            numOfParentsToIncreaseRadius = CN_PTD_NOFPTOINCRAD;
        } else {
            numOfParentsToIncreaseRadius = serialize<float>(element);
        }
        searchParams_[CN_PT_NOP] = numOfParentsToIncreaseRadius;

    TiXmlElement* options = getElement(root, CNS_TAG_OPTIONS);

    element = getElement(options, CNS_TAG_LOGLVL, CNS_TAG_OPTIONS);
    
    loglevel = serialize<float>(element);

    searchParams_[CN_PT_LOGLVL] = loglevel;
}
