#include "detournavigation.h"
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/navigation_mesh.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/variant/builtin_types.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include <climits>
#include <DetourCrowd.h>
#include "util/detourinputgeometry.h"
#include "util/recastcontext.h"
#include "util/godotdetourdebugdraw.h"
#include "util/navigationmeshhelpers.h"
#include "detourobstacle.h"

using namespace godot;

#define SAVE_DATA_VERSION 1

void
DetourNavigationParameters::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_nav_mesh_parameters", "params"), &DetourNavigationParameters::set_nav_mesh_parameters);
    ClassDB::bind_method(D_METHOD("get_nav_mesh_parameters"), &DetourNavigationParameters::get_nav_mesh_parameters);
    ClassDB::bind_method(D_METHOD("set_ticks_per_second", "tps"), &DetourNavigationParameters::set_ticks_per_second);
    ClassDB::bind_method(D_METHOD("get_ticks_per_second"), &DetourNavigationParameters::get_ticks_per_second);
    ClassDB::bind_method(D_METHOD("set_max_obstacles", "max"), &DetourNavigationParameters::set_max_obstacles);
    ClassDB::bind_method(D_METHOD("get_max_obstacles"), &DetourNavigationParameters::get_max_obstacles);

    ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "nav_mesh_parameters"), "set_nav_mesh_parameters", "get_nav_mesh_parameters");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "ticks_per_second"), "set_ticks_per_second", "get_ticks_per_second");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "max_obstacles"), "set_max_obstacles", "get_max_obstacles");
}

void
DetourNavigation::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("initialize"), &DetourNavigation::initialize);
    ClassDB::bind_method(D_METHOD("rebuild_changed_tiles"), &DetourNavigation::rebuildChangedTiles);
    ClassDB::bind_method(D_METHOD("mark_convex_area"), &DetourNavigation::markConvexArea);
    ClassDB::bind_method(D_METHOD("add_agent"), &DetourNavigation::addAgent);
    ClassDB::bind_method(D_METHOD("remove_agent"), &DetourNavigation::removeAgent);
    ClassDB::bind_method(D_METHOD("add_box_obstacle"), &DetourNavigation::addBoxObstacle);
    ClassDB::bind_method(D_METHOD("add_cylinder_obstacle"), &DetourNavigation::addCylinderObstacle);
    ClassDB::bind_method(D_METHOD("create_debug_mesh"), &DetourNavigation::createDebugMesh);
    ClassDB::bind_method(D_METHOD("set_query_filter"), &DetourNavigation::setQueryFilter);
    ClassDB::bind_method(D_METHOD("save"), &DetourNavigation::save);
    ClassDB::bind_method(D_METHOD("load"), &DetourNavigation::load);
    ClassDB::bind_method(D_METHOD("clear"), &DetourNavigation::clear);
    ClassDB::bind_method(D_METHOD("get_agents"), &DetourNavigation::getAgents);
    ClassDB::bind_method(D_METHOD("get_obstacles"), &DetourNavigation::getObstacles);
    ClassDB::bind_method(D_METHOD("get_marked_area_ids"), &DetourNavigation::getMarkedAreaIDs);
    ClassDB::bind_method(D_METHOD("is_initialized"), &DetourNavigation::isInitialized);
    ClassDB::bind_method(D_METHOD("add_off_mesh_connection"), &DetourNavigation::addOffMeshConnection);
    ClassDB::bind_method(D_METHOD("remove_off_mesh_connection"), &DetourNavigation::removeOffMeshConnection);

    ADD_SIGNAL(MethodInfo("navigation_tick_done", PropertyInfo(Variant::INT, "execution_time_seconds")));
}

DetourNavigation::DetourNavigation()
    : _inputGeometry(nullptr)
    , _recastContext(nullptr)
    , _debugDrawer(nullptr)
    , _initialized(false)
    , _ticksPerSecond(60)
    , _maxObstacles(256)
    , _defaultAreaType(0)
    , _navigationThread(nullptr)
    , _stopThread(false)
    , _navigationMutex(nullptr)
{
    _navigationMutex = new std::mutex();
    _recastContext = new RecastContext();
    _inputGeometry = new DetourInputGeometry();
}

DetourNavigation::~DetourNavigation()
{
    _stopThread = true;
    if (_navigationThread)
    {
        if (_navigationThread->joinable())
        {
            _navigationThread->join();
        }
        delete _navigationThread;
    }
    delete _navigationMutex;

    for (const auto& _navMesh : _navMeshes)
    {
        delete _navMesh;
    }
    _navMeshes.clear();

    delete _debugDrawer;
    delete _inputGeometry;
    delete _recastContext;
}

bool
DetourNavigation::initialize(Variant inputMeshInstance, Ref<DetourNavigationParameters> parameters)
{
    // Don't do anything if already initialized
    if (_initialized)
    {
        ERR_PRINT("DetourNavigation already initialized.");
        return false;
    }

    // Make sure we got the input we need
    MeshInstance3D* meshInstance = Object::cast_to<MeshInstance3D>(inputMeshInstance.operator Object*());
    if (meshInstance == nullptr)
    {
        ERR_PRINT("Passed inputMesh must be of type Mesh or MeshInstance.");
        return false;
    }

    // Check if the mesh instance actually has a mesh
    Ref<Mesh> meshToConvert = meshInstance->get_mesh();
    if (meshToConvert.ptr() == nullptr)
    {
        ERR_PRINT("Passed MeshInstance does not have a mesh.");
        return false;
    }

    // Create the input geometry from the passed mesh
    if (!_inputGeometry->loadMesh(_recastContext, meshInstance))
    {
        ERR_PRINT("Input geometry failed to load the mesh.");
        return false;
    }

    // Initialize the navigation mesh(es)
    _ticksPerSecond = parameters->ticksPerSecond;
    _maxObstacles = parameters->maxObstacles;
    _defaultAreaType = parameters->defaultAreaType;
    for (int i = 0; i < parameters->navMeshParameters.size(); ++i)
    {
        Ref<DetourNavigationMeshParameters> navMeshParams = parameters->navMeshParameters[i];
        auto* navMesh = new DetourNavigationMesh();

        if (!navMesh->initialize(_inputGeometry, navMeshParams, _maxObstacles, _recastContext, i))
        {
            ERR_PRINT("Unable to initialize detour navigation mesh!");
            return false;
        }
        _navMeshes.push_back(navMesh);
    }

    // Start the navigation thread
    _stopThread = false;
    _navigationThread = new std::thread(&DetourNavigation::navigationThreadFunction, this);

    _initialized = true;
    return true;
}

void
DetourNavigation::rebuildChangedTiles()
{
    _navigationMutex->lock();
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        _navMeshes[i]->rebuildChangedTiles(_removedMarkedAreaIDs, _removedOffMeshConnections);
    }
    _removedMarkedAreaIDs.clear();
    _removedOffMeshConnections.clear();

    // Mark the volumes as handled
    int volumeCount = _inputGeometry->getConvexVolumeCount();
    for (int i = 0; i < volumeCount; ++i)
    {
        // Get volume
        ConvexVolume volume = _inputGeometry->getConvexVolumes()[i];
        volume.isNew = false;
    }

    // Mark the connections as handled
    int connectionCount = _inputGeometry->getOffMeshConnectionCount();
    for (int i = 0; i < connectionCount; ++i)
    {
        _inputGeometry->getOffMeshConnectionNew()[i] = false;
    }
    _navigationMutex->unlock();
}

int
DetourNavigation::markConvexArea(Array vertices, float height, unsigned int areaType)
{
    // Sanity checks
    if (areaType > UCHAR_MAX)
    {
        ERR_PRINT(String("Passed areaType is too large. {0} (of max allowed {1}).").format(Array::make(areaType, UCHAR_MAX)));
        return -1;
    }
    if (_inputGeometry->getConvexVolumeCount() >= (DetourInputGeometry::MAX_VOLUMES - 1))
    {
        ERR_PRINT("Cannot mark any more convex area, limit reached.");
        return -1;
    }

    // Create the vertices array
    float* vertArray = new float[vertices.size() * 3];
    float miny = 10000000.0f;
    for (int i = 0; i < vertices.size(); ++i)
    {
        Vector3 vertex = vertices[i];
        vertArray[i * 3 + 0] = vertex.x;
        vertArray[i * 3 + 1] = vertex.y;
        vertArray[i * 3 + 2] = vertex.z;

        if (vertex.y < miny)
        {
            miny = vertex.y;
        }
    }

    // Add to the input geometry
    _inputGeometry->addConvexVolume(vertArray, vertices.size(), miny, miny + height, areaType);
    delete [] vertArray;
    int id = _inputGeometry->getConvexVolumeCount() - 1;
    _markedAreaIDs.push_back(id);
    return id;
}

void
DetourNavigation::removeConvexAreaMarker(int id)
{
    _inputGeometry->deleteConvexVolume(id);
    for (int i = 0; i < _markedAreaIDs.size(); ++i)
    {
        if (_markedAreaIDs[i] == id)
        {
            _markedAreaIDs.erase(_markedAreaIDs.begin() +i);
            break;
        }
    }

    // Remember this removal to be able to pass it on to rebuildChangedTiles() later on
    _removedMarkedAreaIDs.push_back(id);
}

int
DetourNavigation::addOffMeshConnection(Vector3 from, Vector3 to, bool bidirectional, float radius, int areaType)
{
    // Sanity checks
    if (_offMeshConnections.size() >= DetourInputGeometry::MAX_OFFMESH_CONNECTIONS)
    {
        ERR_PRINT("Cannot add any more off-mesh connections. Limit reached.");
        return -1;
    }

    // Create parameters
    float start[3];
    float end[3];
    start[0] = from.x;
    start[1] = from.y;
    start[2] = from.z;
    end[0] = to.x;
    end[1] = to.y;
    end[2] = to.z;
    unsigned short flags;
    switch (areaType) {
        case POLY_AREA_GROUND:
        case POLY_AREA_ROAD:
        case POLY_AREA_GRASS:
            flags = POLY_FLAGS_WALK;
            break;
        case POLY_AREA_DOOR:
            flags = POLY_FLAGS_DOOR;
            break;
        case POLY_AREA_WATER:
            flags = POLY_FLAGS_SWIM;
            break;
        case POLY_AREA_JUMP:
            flags = POLY_AREA_JUMP;
            break;
        default:
        {
            ERR_PRINT(String("Unable to add off-mesh connection. Unknown area type: {0}").format(Array::make(areaType)));
            return -1;
        }
    }

    // Add the connection
    _inputGeometry->addOffMeshConnection(start, end, radius, bidirectional, areaType, flags);
    int id = _inputGeometry->getOffMeshConnectionCount() - 1;
    _offMeshConnections.push_back(id);
    return id;
}

void
DetourNavigation::removeOffMeshConnection(int id)
{
    _inputGeometry->deleteOffMeshConnection(id);
    for (int i = 0; i < _offMeshConnections.size(); ++i)
    {
        if (_offMeshConnections[i] == id)
        {
            _offMeshConnections.erase(_offMeshConnections.begin() + i);
            break;
        }
    }

    // Remember this removal to be able to pass it on to rebuildChangedTiles() later on
    _removedOffMeshConnections.push_back(id);
}

bool
DetourNavigation::setQueryFilter(int index, String name, Dictionary weights)
{
    // Check index
    if (index >= 16)
    {
        ERR_PRINT(String("Index exceeds allowed number of query filters: {0}").format(Array::make(index)));
        return false;
    }

    // Set weights
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        dtCrowd* crowd = _navMeshes[i]->getCrowd();
        dtQueryFilter* filter = crowd->getEditableFilter(index);

        for (int j = 0; j < weights.keys().size(); ++j)
        {
            int areaIndex = weights.keys()[j];
            float weight = weights[weights.keys()[j]];
            filter->setAreaCost(areaIndex, weight);

            if (weight > 10000.0f)
            {
                switch (areaIndex)
                {
                case POLY_AREA_WATER:
                    filter->setExcludeFlags(filter->getExcludeFlags() ^ POLY_FLAGS_SWIM);
                    break;
                case POLY_AREA_JUMP:
                    filter->setExcludeFlags(filter->getExcludeFlags() ^ POLY_FLAGS_JUMP);
                    break;
                case POLY_AREA_DOOR:
                    filter->setExcludeFlags(filter->getExcludeFlags() ^ POLY_FLAGS_DOOR);
                    break;
                case POLY_AREA_GRASS:
                case POLY_AREA_GROUND:
                case POLY_AREA_ROAD:
                    filter->setExcludeFlags(filter->getExcludeFlags() ^ POLY_FLAGS_WALK);
                    break;
                }

            }

        }
    }

    // Assign name
    _queryFilterIndices[name] = index;

    return true;
}

Ref<DetourCrowdAgent> DetourNavigation::addAgent(Ref<DetourCrowdAgentParameters> parameters)
{
    _navigationMutex->lock();

    // Find the correct crowd based on the parameters
    DetourNavigationMesh* navMesh = nullptr;
    float bestFitFactor = 10000.0f;
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        float fitFactor = _navMeshes[i]->getActorFitFactor(parameters->radius, parameters->height);
        if (fitFactor > 0.0f && fitFactor < bestFitFactor)
        {
            bestFitFactor = fitFactor;
            navMesh = _navMeshes[i];
        }
    }

    // Make sure we got something
    if (navMesh == nullptr)
    {
        ERR_PRINT(String("Unable to add agent: Too big for any crowd: radius: {0} width: {1}").format(Array::make(parameters->radius, parameters->height)));
        _navigationMutex->unlock();
        return nullptr;
    }

    // Make sure the agent uses a known filter
    if (_queryFilterIndices.find(parameters->filterName) == _queryFilterIndices.end())
    {
        ERR_PRINT(String("Unable to add agent: Unknown filter: {0}").format(Array::make(parameters->filterName)));
        _navigationMutex->unlock();
        return nullptr;
    }

    // Create and add the agent as main
    Ref<DetourCrowdAgent> agent = memnew(DetourCrowdAgent);
    if (!navMesh->addAgent(agent, parameters))
    {
        ERR_PRINT("Unable to add agent.");
        _navigationMutex->unlock();
        return nullptr;
    }
    agent->setFilter(_queryFilterIndices[parameters->filterName]);

    // Add the agent's shadows
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        if (_navMeshes[i] != navMesh)
        {
            if(!_navMeshes[i]->addAgent(agent, parameters, false))
            {
                ERR_PRINT(String("Unable to add agent's shadow: {0}.").format(Array::make(i)));
                _navigationMutex->unlock();
                return nullptr;
            }
        }
    }

    // Add to our list of agents
    _agents.push_back(agent);

    _navigationMutex->unlock();
    return agent;
}


void
DetourNavigation::removeAgent(Ref<DetourCrowdAgent> agent)
{
    _navigationMutex->lock();

    // Agents should not be removed while the nav thread is busy
    // Thus this function is used instead of exposing destroy() to GDScript
    if (agent != nullptr)
    {
      agent->destroy();
    }

    // Remove from the vector
    for (int i = 0; i < _agents.size(); ++i)
    {
        if (_agents[i] == agent)
        {
            _agents.erase(_agents.begin() + i);
            break;
        }
    }

    _navigationMutex->unlock();
}

Ref<DetourObstacle>
DetourNavigation::addCylinderObstacle(Vector3 position, float radius, float height)
{
    _navigationMutex->lock();

    // Create the obstacle
    Ref<DetourObstacle> obstacle = memnew(DetourObstacle);
    obstacle->initialize(OBSTACLE_TYPE_CYLINDER, position, Vector3(radius, height, 0.0f), 0.0f);

    // Add the obstacle to all navmeshes
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        _navMeshes[i]->addObstacle(obstacle);
    }

    _obstacles.push_back(obstacle);
    _navigationMutex->unlock();
    return obstacle;
}

Ref<DetourObstacle>
DetourNavigation::addBoxObstacle(Vector3 position, Vector3 dimensions, float rotationRad)
{
    _navigationMutex->lock();

    // Create the obstacle
    Ref<DetourObstacle> obstacle = memnew(DetourObstacle);
    obstacle->initialize(OBSTACLE_TYPE_BOX, position, dimensions, rotationRad);

    // Add the obstacle to all navmeshes
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        _navMeshes[i]->addObstacle(obstacle);
    }

    _obstacles.push_back(obstacle);
    _navigationMutex->unlock();
    return obstacle;
}

MeshInstance3D*
DetourNavigation::createDebugMesh(int index, bool drawCacheBounds)
{
    _navigationMutex->lock();

    // Sanity check
    if (index > _navMeshes.size() - 1)
    {
        ERR_PRINT(String("Index higher than number of available navMeshes: {0} {1}").format(Array::make(index, _navMeshes.size())));
        _navigationMutex->unlock();
        return nullptr;
    }

    // Create the debug drawing object if it doesn't exist yet
    if (!_debugDrawer)
    {
        _debugDrawer = new GodotDetourDebugDraw();
    }
    //_debugDrawer->setMaterial(material);

    // Get the navmesh
    DetourNavigationMesh* navMesh = _navMeshes[index];

    // Create the debug mesh
    _debugDrawer->clear();
    navMesh->createDebugMesh(_debugDrawer, drawCacheBounds);

    // Add the result to the MeshInstance and return it
    MeshInstance3D* meshInst = memnew(MeshInstance3D);
    meshInst->set_mesh(_debugDrawer->getArrayMesh());

    _navigationMutex->unlock();
    return meshInst;
}

bool
DetourNavigation::save(String path, bool compressed)
{
    // Sanity check
    if (!_initialized)
    {
        ERR_PRINT("DTNavSave: Unable to save navigation data. Navigation not initialized.");
        return false;
    }

    // Open the file
    Ref<FileAccess> saveFile;
    Error result;
    Ref<DirAccess> dir = memnew(DirAccess);
    result = dir->make_dir_recursive(path.left(path.rfind("/")));
    if (result != Error::OK)
    {
        ERR_PRINT(String("DTNavSave: Error while creating navigation file path: {0} {1}").format(Array::make(path, (int)result)));
        return false;
    }
    if (compressed)
    {
        saveFile = FileAccess::open_compressed(path, FileAccess::WRITE, FileAccess::COMPRESSION_ZSTD);
    }
    else
    {
        saveFile = FileAccess::open(path, FileAccess::WRITE);
    }
    result = saveFile->get_open_error();
    if (result != Error::OK)
    {
        ERR_PRINT(String("DTNavSave: Error while opening navigation save file: {0} {1}").format(Array::make(path, (int)result)));
        return false;
    }

    // Version
    saveFile->store_16(SAVE_DATA_VERSION);

    _navigationMutex->lock();

    // Input geometry
    if (!_inputGeometry->save(saveFile))
    {
        ERR_PRINT("DTNavSave: Unable to save input geometry.");
        _navigationMutex->unlock();
        return false;
    }

    // Navmeshes
    saveFile->store_32(_navMeshes.size());
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        if (!_navMeshes[i]->save(saveFile))
        {
            ERR_PRINT(String("DTNavSave: Unable to save nav mesh {0}").format(Array::make(i)));
            _navigationMutex->unlock();
            return false;
        }
    }

    // Query filters
    saveFile->store_32(_queryFilterIndices.size());
    for (const auto& entry : _queryFilterIndices)
    {
        int index = entry.second;
        saveFile->store_pascal_string(entry.first);
        saveFile->store_32(entry.second);

        dtCrowd* crowd = _navMeshes[0]->getCrowd();
        dtQueryFilter* filter = crowd->getEditableFilter(index);
        saveFile->store_16(filter->getExcludeFlags());
        for (int i = 0; i < DT_MAX_AREAS; ++i)
        {
            saveFile->store_float(filter->getAreaCost(i));
        }
    }

    // Agents
    saveFile->store_32(_agents.size());
    for (int i = 0; i < _agents.size(); ++i)
    {
        if (!_agents[i]->save(saveFile))
        {
            ERR_PRINT(String("DTNavSave: Unable to save nav agent {0}").format(Array::make(i)));
            _navigationMutex->unlock();
            return false;
        }
    }

    // Obstacles
    saveFile->store_32(_obstacles.size());
    for (int i = 0; i < _obstacles.size(); ++i)
    {
        if (!_obstacles[i]->save(saveFile))
        {
            ERR_PRINT(String("DTNavSave: Unable to save obstacle {0}").format(Array::make(i)));
            _navigationMutex->unlock();
            return false;
        }
    }

    // Marked area IDs
    saveFile->store_32(_markedAreaIDs.size());
    for (int i = 0; i < _markedAreaIDs.size(); ++i)
    {
        saveFile->store_32(_markedAreaIDs[i]);
    }

    // Off-mesh connections
    saveFile->store_32(_offMeshConnections.size());
    for (int i = 0; i < _offMeshConnections.size(); ++i)
    {
        saveFile->store_32(_offMeshConnections[i]);
    }

    _navigationMutex->unlock();

    saveFile->close();

    return true;
}

bool
DetourNavigation::load(String path, bool compressed)
{
    // Sanity check
    if (_initialized)
    {
        ERR_PRINT("DTNavLoad: Unable to load new navigation data. Navigation still running, please use clear().");
        return false;
    }

    // Load the file
    Ref<FileAccess> saveFile;
    Error result;
    if (compressed)
    {
        saveFile = FileAccess::open_compressed(path, FileAccess::READ, FileAccess::COMPRESSION_ZSTD);
    }
    else
    {
        saveFile = FileAccess::open(path, FileAccess::READ);
    }
    result = saveFile->get_open_error();
    if (result != Error::OK)
    {
        ERR_PRINT(String("DTNavLoad: Error while opening navigation save file: {0} {1}").format(Array::make(path, (int)result)));
        return false;
    }

    // Version
    int version = saveFile->get_16();
    if (version == SAVE_DATA_VERSION)
    {
        // Input geometry
        if (!_inputGeometry->load(saveFile))
        {
            ERR_PRINT("DTNavLoad: Unable to load input geometry.");
            return false;
        }

        // Navmesh(es)
        int numNavMeshes = saveFile->get_32();
        for (int i = 0; i < numNavMeshes; ++i)
        {
            DetourNavigationMesh* navMesh = memnew(DetourNavigationMesh);
            if (!navMesh->load(_inputGeometry, _recastContext, saveFile))
            {
                ERR_PRINT("DTNavLoad: Unable to load navmesh.");
                delete navMesh;
                return false;
            }
            _navMeshes.push_back(navMesh);
        }

        // Query filters
        int numQueryFilters = saveFile->get_32();
        for (int i = 0; i < numQueryFilters; ++i)
        {
            // Get name & index
            String name = saveFile->get_pascal_string();
            int index = saveFile->get_32();
            _queryFilterIndices[name] = index;

            // Get filter values
            int excludeFlags = saveFile->get_16();
            float areaCosts[DT_MAX_AREAS];
            for (int j = 0; j < DT_MAX_AREAS; ++j)
            {
                areaCosts[j] = saveFile->get_float();
            }

            // Apply filter across navmeshes
            for (int j = 0; j < _navMeshes.size(); ++j)
            {
                dtCrowd* crowd = _navMeshes[j]->getCrowd();
                dtQueryFilter* filter = crowd->getEditableFilter(index);
                filter->setExcludeFlags(excludeFlags);
                filter->setAreaCost(j, areaCosts[j]);
            }
        }

        // Agents
        int numAgents = saveFile->get_32();
        for (int i = 0; i < numAgents; ++i)
        {
            Ref<DetourCrowdAgent> agent = memnew(DetourCrowdAgent);
            if (!agent->load(saveFile))
            {
                ERR_PRINT("DTNavLoad: Unable to load agent.");
                return false;
            }

            // Load parameter values
            Ref<DetourCrowdAgentParameters> params = memnew(DetourCrowdAgentParameters);
            if (!agent->loadParameterValues(params, saveFile))
            {
                ERR_PRINT("DTNavLoad: Unable to load agent parameter values.");
                return false;
            }

            // Fully apply the agent
            for (int j = 0; j < numNavMeshes; ++j)
            {
                bool isMain = j == agent->getCrowdIndex();
                if (!_navMeshes[j]->addAgent(agent, params, isMain))
                {
                    ERR_PRINT("DTNavLoad: Unable to add loaded agent via navmesh.");
                    return false;
                }
            }
            agent->setFilter(agent->getFilterIndex());

            // Request movement for the target if it was moving (loading agent resent some states so movement has to be requested again)
            if (agent->isMoving())
            {
                agent->moveTowards(agent->getTargetPosition());
            }

            _agents.push_back(agent);
        }

        // Obstacles
        int numObstacles = saveFile->get_32();
        for (int i = 0; i < numObstacles; ++i)
        {
            Ref<DetourObstacle> obstacle = memnew(DetourObstacle);
            if (!obstacle->load(saveFile))
            {
                ERR_PRINT(String("DTNavLoad: Unable to load obstacle {0}").format(Array::make(i)));
                return false;
            }

            // Add the obstacle to all navmeshes
            for (int i = 0; i < _navMeshes.size(); ++i)
            {
                _navMeshes[i]->addObstacle(obstacle);
            }
            _obstacles.push_back(obstacle);
        }

        // Marked area IDs
        int numMarkedAreaIds = saveFile->get_32();
        for (int i = 0; i < numMarkedAreaIds; ++i)
        {
            _markedAreaIDs.push_back(saveFile->get_32());
        }

        // Off-mesh connections
        int numConnections = saveFile->get_32();
        for (int i = 0; i < numConnections; ++i)
        {
            _offMeshConnections.push_back(saveFile->get_32());
        }
    }
    else
    {
        ERR_PRINT(String("DTNavLoad: Unknown version {0}").format(Array::make(version)));
        return false;
    }

    // Done. Start the thread.
    _stopThread = false;
    _navigationThread = new std::thread(&DetourNavigation::navigationThreadFunction, this);
    _initialized = true;
    return true;
}

void
DetourNavigation::clear()
{
    // Stop the thread
    _stopThread = true;
    if (_navigationThread)
    {
        if (_navigationThread->joinable())
        {
            _navigationThread->join();
        }
        delete _navigationThread;
    }
    _navigationThread = nullptr;

    // Remove all agents
    for (int i = 0; i < _agents.size(); ++i)
    {
        _agents[i]->destroy();
    }
    _agents.clear();

    // Remove all obstacles
    for (int i = 0; i < _obstacles.size(); ++i)
    {
        _obstacles[i]->destroy();
    }
    _obstacles.clear();

    // Remove all marked areas
    for (int i = 0; i < _markedAreaIDs.size(); ++i)
    {
        removeConvexAreaMarker(_markedAreaIDs[i]);
    }
    _markedAreaIDs.clear();

    // Clear the input geometry data
    _inputGeometry->clearData();

    // Free the navigation meshes
    for (int i = 0; i < _navMeshes.size(); ++i)
    {
        delete _navMeshes[i];
    }
    _navMeshes.clear();

    // Other misc stuff
    _queryFilterIndices.clear();
    _initialized = false;
}

Array
DetourNavigation::getAgents()
{
    Array result;

    for (int i = 0; i < _agents.size(); ++i)
    {
        result.append(_agents[i]);
    }

    return result;
}

Array
DetourNavigation::getObstacles()
{
    Array result;

    for (int i = 0; i < _obstacles.size(); ++i)
    {
        if (_obstacles[i]->isDestroyed())
        {
            continue;
        }
        result.append(_obstacles[i]);
    }

    return result;
}

Array
DetourNavigation::getMarkedAreaIDs()
{
    Array result;

    for (int i = 0; i < _markedAreaIDs.size(); ++i)
    {
        result.append(_markedAreaIDs[i]);
    }

    return result;
}

void
DetourNavigation::navigationThreadFunction()
{
    UtilityFunctions::print("DTNav: Navigation thread started");
    double lastExecutionTime = 0.0;
    double secondsToSleepPerFrame = 1.0 / _ticksPerSecond;
    int64_t millisecondsToSleep = 0;
    auto start = std::chrono::system_clock::now();
    while (!_stopThread)
    {
        millisecondsToSleep = (secondsToSleepPerFrame - lastExecutionTime) * 1000.0 + 0.5;
        if (millisecondsToSleep > 0.0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(millisecondsToSleep));
        }

        start = std::chrono::system_clock::now();
        _navigationMutex->lock();

        // Remove obstacles from list if they were destroyed
        for (int i = 0; i < _obstacles.size(); ++i)
        {
            if (_obstacles[i]->isDestroyed())
            {
                _obstacles.erase(_obstacles.begin() + i);
                i--;
            }
        }

        // Apply new movement requests (won't do anything if there's no new target)
        for (int i = 0; i < _agents.size(); ++i)
        {
            _agents[i]->applyNewTarget();
        }

        // Update the navmeshes
        for (int i = 0; i < _navMeshes.size(); ++i)
        {
            _navMeshes[i]->update(secondsToSleepPerFrame);
        }

        // Update the agents
        for (int i = 0; i < _agents.size(); ++i)
        {
            _agents[i]->update(secondsToSleepPerFrame);
        }

        _navigationMutex->unlock();

        // Calculate how long the calculations took and emit the done signal
        auto timeTaken = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
        lastExecutionTime = timeTaken / 1000.0;

        call_deferred("emit_signal", "navigation_tick_done", lastExecutionTime);
    }
    UtilityFunctions::print("DTNav: Navigation thread ended");
}
