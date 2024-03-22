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
    std::string urdf_filepath = "../robot_files/franka/franka.xml";
	std::vector<std::string> mesh_filepaths;

	// ... set up your robot model ...
	parse_urdf(urdf_filepath, robot);

	std::vector<urdf::LinkSharedPtr> links;

	const auto root = robot->getRoot();

    std::string delimiter = "package://";

    robot->getLinks(links);
	PRINT(links.size());

	// get mesh filepaths
	for (auto link : links)
	{
		std::shared_ptr<urdf::Mesh> mesh = nullptr;
		try
		{
			mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
		}
		catch (const std::bad_cast& e)
		{
			std::cout << "bad cast" << std::endl;
		}
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

		PxDefaultFileInputData file(("../PxMeshes/" + links[i]->name + "_convex.bin").c_str());
		std::vector<PxU8> buffer(file.getLength());
		file.read(buffer.data(), file.getLength());

		PxDefaultMemoryInputData inStream(buffer.data(), buffer.size());

		PxConvexMesh* convex = gPhysics->createConvexMesh(inStream);
		PX_ASSERT(convex);

		//create convex geometry
		PxConvexMeshGeometry convexGeom(convex);
		PRINT("is valid?: " + std::to_string(convexGeom.isValid()))
		linkGeometrys.push_back(convexGeom);
	}
}

void test(urdf::ModelInterfaceSharedPtr robot, std::vector<PxConvexMeshGeometry>& linkGeometrys, std::vector<PxShape*>& shapes, std::vector<PxTransform>& poses, std::vector<PxVec3>& axes)
{
	std::vector<urdf::LinkSharedPtr> links;
	robot->getLinks(links);

	for (int i=0; i < links.size(); i++)
	{	auto link = links[i];
		auto linkGeom = linkGeometrys[i];
		PxShape* shape = gPhysics->createShape(linkGeom, *gMaterial);
		shapes.push_back(shape);

		if (!link->child_joints.empty())
		{
			for (const auto& joint : link->child_joints)
			{
				urdf::Pose jointPose = joint->parent_to_joint_origin_transform;
				urdf::Rotation jointRotation = jointPose.rotation;
				PxVec3 jointPosition(jointPose.position.x, jointPose.position.y, jointPose.position.z);
				PxQuat jointQuat(jointRotation.x, jointRotation.y, jointRotation.z, jointRotation.w);
				PxTransform jointTransform(jointPosition, jointQuat);
				poses.push_back(jointTransform);
				PxVec3 jointAxis(joint->axis.x, joint->axis.y, joint->axis.z);
				axes.push_back(jointAxis);
			}
		}
	}
}


PxQuat vecRot(PxVec3 current_axis, PxVec3 target_axis)
{
	current_axis.normalize();
	target_axis.normalize();

	double cos_theta = current_axis.dot(target_axis);
	double theta = std::acos(cos_theta);

	PxVec3 rotation_axis = current_axis.cross(target_axis);
	rotation_axis.normalize();

	PxQuat quat = PxQuat(theta, rotation_axis);
	return quat;
}


PxTransform vecRotX(PxVec3 target_axis)
{
	PxVec3 current_axis(1, 0, 0);
	target_axis.normalize();

	double cos_theta = current_axis.dot(target_axis);
	double theta = std::acos(cos_theta);

	PxVec3 rotation_axis = current_axis.cross(target_axis);
	rotation_axis.normalize();

	PxQuat quat = PxQuat(theta, rotation_axis);
	return PxTransform(quat);
}

void printPose(PxTransform pose)
{
	std::cout << "Position: " << pose.p.x << " " << pose.p.y << " " << pose.p.z << std::endl;
	std::cout << "Quaternion: " << pose.q.w << " " << pose.q.x << " " << pose.q.y << " " << pose.q.z << std::endl;
}


void createArticulation(PxPhysics* gPhysics, PxMaterial* material, PxScene* gScene, std::vector<PxConvexMeshGeometry>& linkGeometrys, urdf::ModelInterfaceSharedPtr robot)
{
	std::vector<PxShape*> shapes;
	std::vector<PxTransform> poses;
	std::vector<PxVec3> axes;
	test(robot, linkGeometrys, shapes, poses, axes);

	PxArticulationReducedCoordinate* articulation = gPhysics->createArticulationReducedCoordinate();


	articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
	articulation->setSolverIterationCounts(32);
	PxTransform identity = PxTransform(PxIdentity);

	PxArticulationLimit limits;
	limits.low = -PxPiDivFour;  // in rad for a rotational motion
	limits.high = PxPiDivFour;

	PxTransform testframe;
	PxMat33 testmat;
	PxVec3 xvec = axes[0];
	PxVec3 yvex = xvec.cross(PxVec3(0,1,0));
	PxVec3 zvec = yvex.cross(xvec);
	testmat.column0 = xvec;
	testmat.column1 = yvex;
	testmat.column2 = zvec;
	testframe.q = PxQuat(testmat);

	PxArticulationLink* base = articulation->createLink(NULL, identity);
	base->attachShape(*shapes[0]);

	PxArticulationLink* link1 = articulation->createLink(base, identity);
	link1->attachShape(*shapes[1]);

	// PxArticulationLink* link2 = articulation->createLink(link1, identity);
	// link2->attachShape(*shapes[2]);
	//
	// PxArticulationLink* link3 = articulation->createLink(link2, identity);
	// link3->attachShape(*shapes[3]);
	//
	// PxArticulationLink* link4 = articulation->createLink(link3, identity);
	// link4->attachShape(*shapes[4]);
	//
	// PxArticulationLink* link5 = articulation->createLink(link4, identity);
	// link5->attachShape(*shapes[5]);
	//
	// PxArticulationLink* link6 = articulation->createLink(link5, identity);
	// link6->attachShape(*shapes[6]);

	PxArticulationJointReducedCoordinate* joint1 = link1->getInboundJoint();
	joint1->setParentPose(base->getGlobalPose().transformInv(poses[0]*vecRotX(axes[0])));
	joint1->setChildPose(PxTransform(vecRotX(axes[0]).q.getConjugate()*poses[0].q));
	joint1->setJointType(PxArticulationJointType::eREVOLUTE);
	joint1->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	joint1->setLimitParams(PxArticulationAxis::eTWIST, limits);

	// PxArticulationJointReducedCoordinate* joint2 = link2->getInboundJoint();
	// joint2->setParentPose(link1->getGlobalPose().transformInv(poses[1]));
	// joint2->setChildPose(identity);
	// joint2->setJointType(PxArticulationJointType::eFIX);
	// // joint2->setJointType(PxArticulationJointType::eREVOLUTE);
	// // joint2->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	// // joint2->setLimitParams(PxArticulationAxis::eTWIST, limits);
	//
	// PxArticulationJointReducedCoordinate* joint3 = link3->getInboundJoint();
	// joint3->setParentPose(link2->getGlobalPose().transformInv(poses[2]*vecRotX(axes[2])));
	// joint3->setChildPose(PxTransform(vecRotX(axes[2]).q.getConjugate()*poses[2].q));
	// joint3->setJointType(PxArticulationJointType::eREVOLUTE);
	// joint3->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	// joint3->setLimitParams(PxArticulationAxis::eTWIST, limits);
	//
	// PxArticulationJointReducedCoordinate* joint4 = link4->getInboundJoint();
	// joint4->setParentPose(link3->getGlobalPose().transformInv(poses[3]*vecRotX(axes[3])));
	// joint4->setChildPose(PxTransform(vecRotX(axes[3]).q.getConjugate()*poses[3].q));
	// joint4->setJointType(PxArticulationJointType::eREVOLUTE);
	// joint4->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	// joint4->setLimitParams(PxArticulationAxis::eTWIST, limits);
	//
	// printPose(poses[4]);
	// printPose(vecRotX(axes[4]));
	// printPose(poses[4]*vecRotX(axes[4]));
	// printPose(link4->getGlobalPose().transformInv(poses[4]*vecRotX(axes[4])));
	//
	// PxArticulationJointReducedCoordinate* joint5 = link5->getInboundJoint();
	// joint5->setParentPose(link4->getGlobalPose().transformInv(poses[4]));
	// joint5->setChildPose(PxTransform(poses[3].q));
	// joint5->setJointType(PxArticulationJointType::eREVOLUTE);
	// joint5->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	// joint5->setLimitParams(PxArticulationAxis::eTWIST, limits);
	//
	// PxArticulationJointReducedCoordinate* joint6 = link6->getInboundJoint();
	// joint6->setParentPose(link5->getGlobalPose().transformInv(poses[5]*vecRotX(axes[5])));
	// joint6->setChildPose(PxTransform(vecRotX(axes[5]).q.getConjugate()*poses[5].q));
	// joint6->setJointType(PxArticulationJointType::eREVOLUTE);
	// joint6->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	// joint6->setLimitParams(PxArticulationAxis::eTWIST, limits);

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
	// sceneDesc.gravity = PxVec3(0.0f, 0.0, 0.0f);
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

	createArticulation(gPhysics, gMaterial, gScene, linkGeometrys, robot);

 	for (int i = 0; i < 100; i++)
	{
		stepPhysics(true);
	}

	cleanupPhysics(false);

	return 0;
}
