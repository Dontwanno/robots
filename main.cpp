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


int loadRobot()
{
    std::string filepath = "../robot_files/urdf.xml";

    //load urdf

    //load parts

    //create decompositions

    //
    return 1;
}

static PxReal stackZ = 10.0f;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
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

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
	}
}

int main(int, const char*const*)
{
	initPhysics(true);

	// check if output_file.bin exists
	if (std::filesystem::exists("../PxBins/output_file.bin")) {
		std::cout << "File exists, not creating new file." << std::endl;
	} else {
		std::cout << "File does not exist" << std::endl;
		createConvex("../robot_files/base_link.stl", gPhysics);
	}

	PxDefaultMemoryInputData inStream = createMemoryInputData("../PxBins/output_file.bin");

	PxConvexMesh* convex = gPhysics->createConvexMesh(inStream);
	PX_ASSERT(convex);

	std::cout << "Convex mesh created." << std::endl;

	//create convex geometry
	PxConvexMeshGeometry convexGeom(convex);


	cleanupPhysics(false);
	return 0;
}
