//
// Created by twang on 1/31/2024.
//

#ifndef COOKING_H
#define COOKING_H

#include <PxPhysicsAPI.h>

physx::PxDefaultMemoryInputData createMemoryInputData(const std::string& fileName);
void createConvex(const std::string& fileName, physx::PxPhysics* gPhysics);

#endif //COOKING_H
