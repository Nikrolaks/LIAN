#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <optional>
#include <map>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <limits>
#include <unordered_map>

template <typename T>
struct vec {
	T y, x;
};

template <typename T, typename XmlElement>
T serialize(XmlElement* element) {
    std::stringstream ss;
    ss << std::string(element->GetText());
    T res;
    ss >> res;
    return res;
}

template <typename T, typename XmlElement>
T serializeOrElse(XmlElement* element, const T& orElse) {
    if (!element) {
        return orElse;
    }
    return serialize<T>(element);
}

template <typename T, typename XmlElement>
std::optional<T> serializeOpt(XmlElement* element) {
    if (!element) {
        return std::nullopt;
    }
    return serialize<T>(element);
}

template <typename T, typename XmlElement>
std::vector<T> serializeVector(XmlElement* element) {
    std::stringstream ss;
    ss << std::string(element->GetText());
    T cell;
    std::vector<T> res;
    while (ss >> cell) {
        res.push_back(cell);
    }

    return res;
}

template <typename XmlElement>
XmlElement* getElement(
    XmlElement* section, const std::string& elementName,
    std::optional<std::string> sectionName = std::nullopt) {
    XmlElement* element = section->FirstChildElement(elementName.c_str());
    if (!element) {
        std::stringstream errorStream;
        errorStream << "Error! No '" << elementName << "' element found";
        if (sectionName) {
            errorStream << " inside '" << (*sectionName) << "' section.";
        }
        throw std::runtime_error(errorStream.str());
    }

    return element;
}
