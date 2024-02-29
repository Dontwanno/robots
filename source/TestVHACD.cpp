#include <cstdio>

#define TEST_FOR_MEMORY_LEAKS 0 // set to 1, on Windows only, to enable memory leak checking on application exit
#if TEST_FOR_MEMORY_LEAKS
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#else
#include <cstdlib>
#endif

#include <cstring>
#include <cstdint>

#define ENABLE_VHACD_IMPLEMENTATION 1
#define VHACD_DISABLE_THREADING 0
#include "VHACD.h"
#include "ScopedTime.h"

#include <thread>
#include <string>
#include <vector>
#include <array>

#ifdef _MSC_VER
#pragma warning(disable:4100 4996)
#include <conio.h>
#endif

class Logging : public VHACD::IVHACD::IUserCallback,
                public VHACD::IVHACD::IUserLogger
{
public:
	Logging(void)
	{
	}

	~Logging(void)
	{
		flushMessages();
	}

        // Be aware that if you are running V-HACD asynchronously (in a background thread) this callback will come from
        // a different thread. So if your print/logging code isn't thread safe, take that into account.
        virtual void Update(const double overallProgress,
                            const double stageProgress,
                            const char* const stage,const char *operation) final
		{
			char scratch[512];
			snprintf(scratch,sizeof(scratch),"[%-40s] : %0.0f%% : %0.0f%% : %s",stage,overallProgress,stageProgress,operation);

			if ( strcmp(stage,mCurrentStage.c_str()) == 0 )
			{
				for (uint32_t i=0; i<mLastLen; i++)
				{
					printf("%c", 8);
				}
			}
			else
			{
				printf("\n");
				mCurrentStage = std::string(stage);
			}
			mLastLen = (uint32_t)strlen(scratch);
			printf("%s", scratch);
		}

        // This is an optional user callback which is only called when running V-HACD asynchronously.
        // This is a callback performed to notify the user that the
        // convex decomposition background process is completed. This call back will occur from
        // a different thread so the user should take that into account.
        virtual void NotifyVHACDComplete(void)
        {
			Log("VHACD::Complete");
        }

		virtual void Log(const char* const msg) final
		{
			mLogMessages.push_back(std::string(msg));
		}

		void flushMessages(void)
		{
			if ( !mLogMessages.empty() )
			{
				printf("\n");
				for (auto &i:mLogMessages)
				{
					printf("%s\n", i.c_str());
				}
				mLogMessages.clear();
			}
		}

		uint32_t	mLastLen{0};
		std::string mCurrentStage;
		std::vector< std::string > mLogMessages;

};

enum class ExportFormat
{
	NONE,
	WAVEFRONT,
	STL,
};

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

int VHACD_create(const std::string& inputFile)
{

#if TEST_FOR_MEMORY_LEAKS
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#endif
	bool showLogging=true;
	Logging logging;
	VHACD::IVHACD::Parameters p;
	p.m_callback = &logging;
	p.m_logger = &logging;

	ExportFormat format = ExportFormat::NONE;

	p.m_maxConvexHulls = uint32_t(32);
	p.m_resolution = uint32_t(100000);
	p.m_minimumVolumePercentErrorAllowed = uint32_t(1);
	// format = ExportFormat::WAVEFRONT;
	format = ExportFormat::STL;
	p.m_maxRecursionDepth = uint32_t(10);
	p.m_shrinkWrap = true;
	showLogging = true;
	if (!showLogging)
	{
		p.m_logger = nullptr;
		p.m_callback = nullptr;
	}
	p.m_fillMode = VHACD::FillMode::FLOOD_FILL;
	// p.m_fillMode = VHACD::FillMode::RAYCAST_FILL;
	// p.m_fillMode = VHACD::FillMode::SURFACE_ONLY;
	p.m_maxNumVerticesPerCH = uint32_t(64);
	p.m_asyncACD = true;
	p.m_minEdgeLength = uint32_t(2);
	p.m_findBestPlane = false;

	// Create an instance of the Importer class
	Assimp::Importer importer;

	// And have it read the given file with some example postprocessing
	// Usually - if speed is not the most important aspect for you - you'll
	// probably to request more postprocessing than we do in this example.
	const struct aiScene* scene = importer.ReadFile( inputFile,
	  aiProcess_CalcTangentSpace       |
	  aiProcess_Triangulate            |
	  aiProcess_JoinIdenticalVertices  |
	  aiProcess_SortByPType);

	if (scene-> mNumMeshes != 1)
	{
		std::cout << "Only one mesh per file is supported." << std::endl;
		return 0;
	}

	aiMesh* mesh = scene->mMeshes[0];

	std::vector<unsigned int> indices(mesh->mNumFaces * 3);

	for (unsigned int f = 0; f < mesh->mNumFaces; ++f)
	{
		// we know we're always working with triangles due to TRIANGULATE option.
		for (unsigned int i = 0; i < 3; ++i)
		{
			indices[f * 3 + i] = mesh->mFaces[f].mIndices[i];
		}
	}

	auto faces = mesh->mFaces;

	VHACD::IVHACD *iface = p.m_asyncACD ? VHACD::CreateVHACD_ASYNC() : VHACD::CreateVHACD();

	double *points = new double[mesh->mNumVertices*3];
	for (uint32_t i=0; i < mesh->mNumVertices*3; i++)
	{
		points[i] = mesh->mVertices[i/3][i%3];
	}

	ScopedTime st("Computing Convex Decomposition");
	iface->Compute(points, mesh->mNumVertices,indices.data(),mesh->mNumFaces,p);
	while ( !iface->IsReady() )
	{
		std::this_thread::sleep_for(std::chrono::nanoseconds(10000)); // s
	}

	logging.flushMessages();
	delete []points;

	std::cout << "Convex Decomposition Complete" << std::endl;
	std::cout << "Number of Convex Hulls: " << iface->GetNConvexHulls() << std::endl;

	iface->Release();
	return 0;
}
