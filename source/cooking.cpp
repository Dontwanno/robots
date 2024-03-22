#include <cctype>
#include <stdio.h>
#include <stdlib.h> // For aligned_alloc

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


void createConvex(const std::string& fileName, const std::string& outputFileName, PxPhysics* gPhysics)
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
	// PxU32* indices = new PxU32[mesh->mNumFaces * 3];
	// std::vector<PxVec3> vertices(mesh->mNumVertices);

	// // Prepare vertices
	// for(PxU32 i = 0; i < mesh->mNumVertices; i++)
	// {
	// 	aiVector3D vertex = mesh->mVertices[i];
	// 	vertices.emplace_back(PxVec3(vertex.x, vertex.y, vertex.z));
	// }
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

		// Open the output file
		PxDefaultFileOutputStream file(("../PxMeshes/" + outputFileName + "_convex.bin").c_str());

		file.write(outStream.getData(), outStream.getSize());

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

	// PxConvexMeshGeometry testgeom(convex);
	// std::cout << "is convex geom valid?? " << testgeom.isValid() << std::endl;

	convex->release();
}

// Setup common cooking params
void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
	// we suppress the triangle mesh remap table computation to gain some speed, as we will not need it
	// in this snippet
	params.suppressTriangleMeshRemapTable = true;

	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid.
	// The following conditions are true for a valid triangle mesh :
	//  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
	//  2. There are no large triangles(within specified PxTolerancesScale.)
	// It is recommended to run a separate validation check in debug/checked builds, see below.

	if (!skipMeshCleanup)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	// If eDISABLE_ACTIVE_EDGES_PRECOMPUTE is set, the cooking does not compute the active (convex) edges, and instead
	// marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change
	// the collision behavior, as all edges of the triangle mesh will now be considered active.
	if (!skipEdgeData)
		params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
	else
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

// Creates a triangle mesh using BVH34 midphase with different settings.
void createTriangleMesh(const std::string& fileName,
	bool skipMeshCleanup, bool skipEdgeData, bool inserted, const PxU32 numTrisPerLeaf, PxPhysics* gPhysics)
{
	// PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices

	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

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

	// PxVec3* vertices = new PxVec3[mesh->mNumVertices];
	// PxU32* indices = new PxU32[mesh->mNumFaces * 3];
	std::vector<PxVec3> vertices(mesh->mNumVertices);
	std::vector<PxU32> indices(mesh->mNumFaces * 3);

	// Prepare vertices
	for(PxU32 i = 0; i < mesh->mNumVertices; i++)
	{
		aiVector3D vertex = mesh->mVertices[i];
		vertices.emplace_back(vertex.x, vertex.y, vertex.z);
	}

	// Prepare indices
	for (int i = 0; i < mesh->mNumFaces; ++i) {
		const aiFace& face = mesh->mFaces[i];
		// Assuming the face has 3 indices (a triangle)
		indices[i * 3 + 0] = face.mIndices[0];
		indices[i * 3 + 1] = face.mIndices[1];
		indices[i * 3 + 2] = face.mIndices[2];
	}

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = mesh->mNumVertices;
	meshDesc.points.data = vertices.data();
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = mesh->mNumFaces;
	meshDesc.triangles.data = indices.data();
	meshDesc.triangles.stride = 3 * sizeof(PxU32);

	PxTolerancesScale scale;
	PxCookingParams params(scale);

	// Create BVH34 midphase
	params.midphaseDesc = PxMeshMidPhase::eBVH34;

	// setup common cooking params
	setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

	// Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
	// and worse cooking performance. Cooking time is better when more triangles per leaf are used.
	params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = numTrisPerLeaf;

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking.
	// We should check the validity of provided triangles in debug/checked builds though.
	if (skipMeshCleanup)
	{
		PX_ASSERT(PxValidateTriangleMesh(params, meshDesc));
	}
#endif // DEBUG


	PxTriangleMesh* triMesh = NULL;
	PxU32 meshSize = 0;

	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (inserted)
	{
		triMesh = PxCreateTriangleMesh(params, meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		PxDefaultMemoryOutputStream outStream;
		PxCookTriangleMesh(params, meshDesc, outStream);

		PxDefaultMemoryInputData stream(outStream.getData(), outStream.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);

		meshSize = outStream.getSize();

		// Get the data from the memory output stream
		const uint8_t* data = outStream.getData();
		size_t dataSize = outStream.getSize();

		// Open the output file
		std::ofstream file("../PxBins/" + fileName + "_triangle.bin", std::ios::out | std::ios::binary);

		// Write to the file
		if (file.is_open()) {
			file.write(reinterpret_cast<const char*>(data), dataSize);
			file.close();
		} else {
			std::cout << "Error: Could not open file for writing." << std::endl;
		}
	}

	// Print the elapsed time for comparison
	PxU64 stopTime = SnippetUtils::getCurrentTimeCounterValue();
	float elapsedTime = SnippetUtils::getElapsedTimeInMilliseconds(stopTime - startTime);
	printf("\t -----------------------------------------------\n");
	printf("\t Create triangle mesh with %d triangles: \n", mesh->mNumFaces);
	inserted ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
	!skipEdgeData ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
	!skipMeshCleanup ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
	printf("\t\t Num triangles per leaf: %d \n", numTrisPerLeaf);
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!inserted)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}

	triMesh->release();
}

