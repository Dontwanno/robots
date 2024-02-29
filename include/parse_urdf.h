//
// Created by twang on 30/01/2024.
//

#ifndef PARSE_URDF_H
#define PARSE_URDF_H

#include <urdfdom/urdf_parser/urdf_parser.h>

int parse_urdf(const std::string& filepath, urdf::ModelInterfaceSharedPtr& robot);

#endif //PARSE_URDF_H
