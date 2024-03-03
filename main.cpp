#include <fstream>
#include <iostream>
#include <PxPhysicsAPI.h>
#include <filesystem>

#include "cooking.h"
#include "parse_urdf.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;


#define PVD_HOST "127.0.0.1"	//Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.

#define PRINT(x) std::cout << x << std::endl;


void loadRobot(std::vector<PxConvexMeshGeometry>& linkGeometrys, urdf::ModelInterfaceSharedPtr& robot)
{
    std::string urdf_filepath = "../robot_files/edo_sim/urdf.xml";
	std::vector<std::string> mesh_filepaths;

	// ... set up your robot model ...
	parse_urdf(urdf_filepath, robot);

	std::vector<urdf::LinkSharedPtr> links;

	const auto root = robot->getRoot();

    std::string delimiter = "package://";

    robot->getLinks(links);

	// get mesh filepaths
	for (auto link : links)
	{
		auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
		if (mesh)
		{
			std::string filename = mesh->filename;
			std::string token = filename.substr(delimiter.length(), filename.length() - 1);
			mesh_filepaths.push_back(token);
		}
	}

	// create convex meshes
	for (int i = 0; i < mesh_filepaths.size(); i++)
	{
		// check if output_file.bin exists
		PRINT("../robot_files/" + mesh_filepaths[i])
		if (std::filesystem::exists("../PxMeshes/" + links[i]->name + "_convex.bin")) {
			std::cout << "File exists, not creating new file." << std::endl;
		} else {
			std::cout << "File does not exist" << std::endl;
			createConvex("../robot_files/" + mesh_filepaths[i], links[i]->name, gPhysics);
		}

		PRINT("../PxMeshes/" + links[i]->name + ".bin")
		PxDefaultMemoryInputData inStream = createMemoryInputData("../PxMeshes/" + links[i]->name + "_convex.bin");

		PRINT("Size is: " << inStream.getLength() << " bytes.");

		PxConvexMesh* convex = gPhysics->createConvexMesh(inStream);
		PX_ASSERT(convex);

		std::cout << "Convex mesh created." << std::endl;

		//create convex geometry
		PxConvexMeshGeometry convexGeom(convex);
		PRINT(convexGeom.isValid())
		linkGeometrys.push_back(convexGeom);
	}
}

void createArticulation(PxPhysics* gPhysics, PxMaterial* material, PxScene* gScene, std::vector<PxConvexMeshGeometry>& linkGeometrys)
{
	PxArticulationReducedCoordinate* articulation = gPhysics->createArticulationReducedCoordinate();

	PRINT(linkGeometrys.size());

	articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
	articulation->setSolverIterationCounts(32);

	PxTransform trans = PxTransform(PxVec3(0.0f, 0.0f, 0.0f));

	PxArticulationLink* link = articulation->createLink(NULL, trans);
	PxRigidActorExt::createExclusiveShape(*link, linkGeometrys[0], *material);
	PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);

	gScene->addArticulation(*articulation);


}


void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetHelloWorld done.\n");
}


int main(int, const char*const*)
{
	urdf::ModelInterfaceSharedPtr robot;
	std::vector<PxConvexMeshGeometry> linkGeometrys;


	initPhysics(true);

	loadRobot(linkGeometrys, robot);

	// createArticulation(gPhysics, gMaterial, gScene, linkGeometrys);

	// while (true)
	// {
	// 	stepPhysics(true);
	// }

	cleanupPhysics(false);

	return 0;
}
