//
// Created by twang on 30/01/2024.
//

#include <fstream>
#include <urdfdom/urdf_parser/urdf_parser.h>
#include "parse_urdf.h"

int parse_urdf(const std::string& filepath, urdf::ModelInterfaceSharedPtr& robot)
{
    std::ifstream file(filepath); // Open the file for reading

    if (!file.is_open()) {
        std::cerr << "Error opening file\n";
        return 1;
    }

    std::string xml_string;
    std::string line;

    // Read lines from the file and output them
    while (std::getline(file, line)) {
        xml_string += line + '\n';
    }

    file.close(); // Close the file after reading

    robot = urdf::parseURDF(xml_string);

    if (!robot){
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return -1;
    }
    std::cout << "robot name is: " << robot->getName() << std::endl;

    // get info from parser
    std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
    // get root link
    urdf::LinkConstSharedPtr root_link=robot->getRoot();
    if (!root_link) return -1;

    std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;

    return 0;
}