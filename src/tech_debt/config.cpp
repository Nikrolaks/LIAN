#include "config.h"

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"


const SearchParams& Config::params() const {
    return params_;
}

const std::string& Config::getMapFileName() const {
    return mapFileName_;
}

Config::Config(const std::string& fileName) {
    TiXmlDocument doc(fileName.c_str());
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file.");
    }

    auto root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        throw std::runtime_error((std::stringstream() << "No '" << CNS_TAG_ROOT << "' element found in XML file.").str());
    }

    auto algorithm = getElement(root, CNS_TAG_ALGORITHM, CNS_TAG_ROOT);

    TiXmlElement *element;

    element = getElement(root, CNS_TAG_MAP);
    mapFileName_ = serialize<std::string>(element);

    element = getElement(algorithm, CNS_TAG_ANGLELIMIT, CNS_TAG_ALGORITHM);
    params_.angleLimit = std::min(std::abs(serialize<double>(element)), 180.0);

    element = getElement(algorithm, CNS_TAG_DISTANCE, CNS_TAG_ALGORITHM);
    params_.baseSegmentLength = std::abs(serialize<double>(element));

    element = algorithm->FirstChildElement("isELian");
    params_.doELian = serializeOrElse<bool>(element, false);

    element = algorithm->FirstChildElement(CNS_TAG_WEIGHT);
    params_.heuristicWeight = serializeOrElse<double>(element, CN_PTD_W);

    element = algorithm->FirstChildElement(CNS_TAG_STEPLIMIT);
    params_.stepsLimit = serializeOpt<uint64_t>(element);
        
    element = algorithm->FirstChildElement(CNS_TAG_CURVHEURWEIGHT);
    params_.curvatureHeuristicWeight = serializeOrElse<double>(element, 0.0);
        
    element = algorithm->FirstChildElement(CNS_TAG_SMOOTHER);
    params_.doSmoothing = serializeOrElse<bool>(element, false);

    element = algorithm->FirstChildElement(CNS_TAG_DECRDISTFACTOR);
    params_.distanceDecreaseCoefficient = serializeOrElse<double>(element, CN_PTD_DDF);

    element = algorithm->FirstChildElement(CNS_TAG_DISTANCEMIN);
    params_.minSegmentLength = serializeOrElse<double>(element, params_.baseSegmentLength * 0.1);

    element = algorithm->FirstChildElement(CNS_TAG_PIVOTCIRCLE);
    params_.pivotCircleRadius = serializeOrElse<double>(element, CN_PT_PC);

    element = algorithm->FirstChildElement(CNS_TAG_NOFPTOINCRAD);
    params_.parentsToIncreaseRadius = serializeOrElse<uint64_t>(element, CN_PTD_NOFPTOINCRAD);

    TiXmlElement* options = getElement(root, CNS_TAG_OPTIONS);

    element = getElement(options, CNS_TAG_LOGLVL, CNS_TAG_OPTIONS);
    
    params_.logLevel = serialize<float>(element);
}
