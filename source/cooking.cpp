#include <ctype.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "PxPhysicsAPI.h"
#include "SnippetUtils.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

using namespace physx;

PxDefaultMemoryInputData createMemoryInputData(const std::string& fileName)
{
	std::ifstream file(fileName, std::ios::in | std::ios::binary);

	if (!file.is_open()) {
		std::cerr << "Error opening file!" << std::endl;
	}

	file.seekg(0, std::ios::end);
	size_t dataSize = file.tellg();
	file.seekg(0, std::ios::beg);

	std::vector<char> buffer(dataSize);

	file.read(buffer.data(), dataSize);
	file.close();

	PxU8* PxData = reinterpret_cast<PxU8*>(buffer.data());
	// Create the mesh from a stream.
	PxDefaultMemoryInputData inStream(PxData, dataSize);

	return inStream;
}


void createConvex(const std::string& fileName, PxPhysics* gPhysics)
{
	bool directInsertion = false;

	// Load the convex mesh data from a file with assimp
	// Create an instance of the Importer class
	Assimp::Importer importer;

	// And have it read the given file with some example postprocessing
	// Usually - if speed is not the most important aspect for you - you'll
	// probably to request more postprocessing than we do in this example.
	const struct aiScene* scene = importer.ReadFile( fileName,
	  aiProcess_CalcTangentSpace       |
	  aiProcess_Triangulate            |
	  aiProcess_JoinIdenticalVertices  |
	  aiProcess_SortByPType);

	if (scene-> mNumMeshes != 1)
	{
		std::cout << "Only one mesh per file is supported." << std::endl;
	}

	aiMesh* mesh = scene->mMeshes[0];

	PxVec3* vertices = new PxVec3[mesh->mNumVertices];
	PxU32* indices = new PxU32[mesh->mNumFaces * 3];

	// Prepare vertices
	for(PxU32 i = 0; i < mesh->mNumVertices; i++)
	{
		aiVector3D vertex = mesh->mVertices[i];
		vertices[i] = PxVec3(vertex.x, vertex.y, vertex.z);
	}

	// // Prepare indices
	// for (int i = 0; i < mesh->mNumFaces; ++i) {
	// 	const aiFace& face = mesh->mFaces[i];
	// 	// Assuming the face has 3 indices (a triangle)
	// 	indices[i * 3 + 0] = face.mIndices[0];
	// 	indices[i * 3 + 1] = face.mIndices[1];
	// 	indices[i * 3 + 2] = face.mIndices[2];
	// }

	PxTolerancesScale tolerances;
	PxCookingParams params(tolerances);

	// Use the new (default) PxConvexMeshCookingType::eQUICKHULL
	params.convexMeshCookingType = PxConvexMeshCookingType::eQUICKHULL;

	// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
	// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
	PxU32 gaussMapLimit = 16;
	// set to 256 for gauss map or something...
	params.gaussMapLimit = gaussMapLimit;

	// Setup the convex mesh descriptor
	PxConvexMeshDesc convexDesc;

	// convexDesc.points.count             = mesh->mNumVertices;
	// convexDesc.points.stride            = sizeof(PxVec3);
	// convexDesc.points.data              = vertices;
	// convexDesc.indices.count			= mesh->mNumFaces;
	// convexDesc.indices.stride			= sizeof(PxU32);
	// convexDesc.indices.data				= indices;

	convexDesc.points.data = vertices;
	convexDesc.points.count = mesh->mNumVertices;
	convexDesc.points.stride = sizeof(PxVec3);
	convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxU32 meshSize = 0;
	PxConvexMesh* convex = NULL;

	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

	if(directInsertion)
	{
		// Directly insert mesh into PhysX
		convex = PxCreateConvexMesh(params, convexDesc, gPhysics->getPhysicsInsertionCallback());
		PX_ASSERT(convex);
	}
	else
	{
		// Serialize the cooked mesh into a stream.
		PxDefaultMemoryOutputStream outStream;
		bool res = PxCookConvexMesh(params, convexDesc, outStream);
		PX_UNUSED(res);
		PX_ASSERT(res);
		meshSize = outStream.getSize();

		// Get the data from the memory output stream
		const uint8_t* data = outStream.getData();
		size_t dataSize = outStream.getSize();

		// Open the output file
		std::ofstream file("../PxMeshes/output_file.bin", std::ios::out | std::ios::binary);

		// Write to the file
		if (file.is_open()) {
			file.write(reinterpret_cast<const char*>(data), dataSize);
			file.close();
		} else {
			std::cout << "Error: Could not open file for writing." << std::endl;
		}

		// Create the mesh from a stream.
		PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
		convex = gPhysics->createConvexMesh(inStream);
		PX_ASSERT(convex);
	}

	// Print the elapsed time for comparison
	PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();
	float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
	printf("\t -----------------------------------------------\n");
	printf("\t Create convex mesh with %d triangles: \n", mesh->mNumFaces);
	directInsertion ? printf("\t\t Direct mesh insertion enabled\n") : printf("\t\t Direct mesh insertion disabled\n");
	printf("\t\t Gauss map limit: %d \n", gaussMapLimit);
	printf("\t\t Created hull number of vertices: %d \n", convex->getNbVertices());
	printf("\t\t Created hull number of polygons: %d \n", convex->getNbPolygons());
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!directInsertion)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}

	delete []vertices;
	delete []indices;

	convex->release();
}