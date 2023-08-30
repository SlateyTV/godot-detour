#ifndef DETOURNAVIGATIONMESH_H
#define DETOURNAVIGATIONMESH_H

#include <godot_cpp/variant/builtin_types.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <vector>
#include "detourcrowdagent.h"

class DetourInputGeometry;
class dtTileCache;
class dtNavMesh;
class dtNavMeshQuery;
class dtCrowd;
class RecastContext;
class GodotDetourDebugDraw;
struct MeshProcess;
struct rcConfig;
struct LinearAllocator;
struct FastLZCompressor;
struct TileCacheData;

namespace godot
{
    class MeshInstance3D;
    class DetourObstacle;
    class FileAccess;

    /**
     * @brief Parameters to initialize a DetourNavigationMesh.
     */
    struct DetourNavigationMeshParameters : public RefCounted
    {
        GDCLASS(DetourNavigationMeshParameters, RefCounted)

    public:
        /**
         * @brief Called when .new() is called in gdscript
         */
        void _init() {}

        static void _bind_methods();

        // It is important to understand that recast/detour operates on a voxel field internally.
        // The size of a single voxel (often called cell internally) has significant influence on how a navigation mesh is created.
        // A tile is a rectangular region within the navigation mesh. In other words, every navmesh is divided into equal-sized tiles, which are in turn divided into cells.
        // The detail mesh is a mesh used for determining surface height on the polygons of the navigation mesh.
        // Units are usually in world units [wu] (e.g. meters, or whatever you use), but some may be in voxel units [vx] (multiples of cellSize).
        Vector2     cellSize;               // x = width & depth of a single cell (only one value as both must be the same) | y = height of a single cell. [wu]
        int         maxNumAgents;           // How many agents this mesh can manage at once.
        float       maxAgentSlope;          // How steep an angle can be to still be considered walkable. In degree. Max 90.0.
        float       maxAgentHeight;         // The maximum height of an agent supported in this navigation mesh. [wu]
        float       maxAgentClimb;          // How high a single "stair" can be to be considered walkable by an agent. [wu]
        float       maxAgentRadius;         // The maximum radius of an agent in this navigation mesh. [wu]
        float       maxEdgeLength;          // The maximum allowed length for contour edges along the border of the mesh. [wu]
        float       maxSimplificationError; // The maximum distance a simplified contour's border edges should deviate the original raw contour. [vx]
        int         minNumCellsPerIsland;   // How many cells an isolated area must at least have to become part of the navmesh.
        int         minCellSpanCount;       // Any regions with a span count smaller than this value will, if possible, be merged with larger regions.
        int         maxVertsPerPoly;        // Maximum number of vertices per polygon in the navigation mesh.
        int         tileSize;               // The width,depth & height of a single tile. [vx]
        int         layersPerTile;          // How many vertical layers a single tile is expected to have. Should be less for "flat" levels, more for something like tall, multi-floored buildings.
        float       detailSampleDistance;   // The sampling distance to use when generating the detail mesh. [wu]
        float       detailSampleMaxError;   // The maximum allowed distance the detail mesh should deviate from the source data. [wu]

    public:
        void set_cell_size(Vector2 size) {
            cellSize = size;
        }
        Vector2 get_cell_size() {
            return cellSize;
        }

        void set_max_num_agents(int agents) {
            maxNumAgents = agents;
        }
        int get_max_num_agents() {
            return maxNumAgents;
        }

        void set_max_agent_slope(float slope) {
            maxAgentSlope = slope;
        }
        float get_max_agent_slope() {
            return maxAgentSlope;
        }

        void set_max_agent_height(float height) {
            maxAgentHeight = height;
        }
        float get_max_agent_height() {
            return maxAgentHeight;
        }

        void set_max_agent_climb(float climb) {
            maxAgentClimb = climb;
        }
        float get_max_agent_climb() {
            return maxAgentClimb;
        }

        void set_max_agent_radius(float radius) {
            maxAgentRadius = radius;
        }
        float get_max_agent_radius() {
            return maxAgentRadius;
        }

        void set_max_edge_length(float length) {
            maxEdgeLength = length;
        }
        float get_max_edge_length() {
            return maxEdgeLength;
        }

        void set_max_simplification_error(float max_error) {
            maxSimplificationError = max_error;
        }
        float get_max_simplification_error() {
            return maxSimplificationError;
        }

        void set_min_num_cells_per_island(int cells) {
            minNumCellsPerIsland = cells;
        }
        int get_min_num_cells_per_island() {
            return minNumCellsPerIsland;
        }

        void set_min_cells_span_count(int count) {
            minCellSpanCount = count;
        }
        int get_min_cells_span_count() {
            return minCellSpanCount;
        }

        void set_max_verts_per_poly(int verts) {
            maxVertsPerPoly = verts;
        }
        int get_max_verts_per_poly() {
            return maxVertsPerPoly;
        }

        void set_tile_size(int size) {
            tileSize = size;
        }
        int get_tile_size() {
            return tileSize;
        }

        void set_layers_per_tile(int layers) {
            layersPerTile = layers;
        }
        int get_layers_per_tile() {
            return layersPerTile;
        }

        void set_detail_sample_distance(float distance) {
            detailSampleDistance = distance;
        }
        float get_detail_sample_distance() {
            return detailSampleDistance;
        }

        void set_detail_sample_max_error(float max_error) {
            detailSampleMaxError = max_error;
        }
        float get_detail_sample_max_error() {
            return detailSampleMaxError;
        }
    };

    // Helper struct to store convex volume data
    struct ConvexVolumeData
    {
        Array           vertices;
        float           height;
        unsigned char   areaType;
    };

    // Helper struct for changed tiles
    struct ChangedTileLayerData
    {
        int                 ref;
        int                 layer;
        bool                doAll;
    };

    // Helper struct to store data about changed tile layers
    struct ChangedTileLayers
    {
        std::pair<int, int>                 tilePos;
        std::vector<ChangedTileLayerData>   layers;
    };

    /**
     * @brief Representation of a single TileMesh and Crowd.
     */
    class DetourNavigationMesh
    {
    public:
        /**
         * @brief Destructor.
         */
        DetourNavigationMesh();

        /**
         * @brief Destructor.
         */
        ~DetourNavigationMesh();

        /**
         * @brief initialize    Initializes the navigation mesh & crowd.
         * @param inputGeom     The input geometry.
         * @param params        The parameters for setting up this navigation mesh + crowd.
         * @param maxObstacles  The maximum amount of obstacles supported.
         * @return True if everything was successful. False otherwise.
         */
        bool initialize(DetourInputGeometry* inputGeom, Ref<DetourNavigationMeshParameters> params, int maxObstacles, RecastContext* recastContext, int index);

        /**
         * @brief Will save this navmesh's current state to the passed file.
         * @param targetFile The file to append data to.
         * @return True if everything worked out, false otherwise.
         */
        bool save(Ref<godot::FileAccess> targetFile);

        /**
         * @brief Loads and initializes the navmesh from the file.
         * @param sourceFile The file to read data from.
         * @return True if everything worked out, false otherwise.
         */
        bool load(DetourInputGeometry* inputGeom, RecastContext* recastContext, Ref<godot::FileAccess> sourceFile);

        /**
         * @brief Rebuilds all tiles that have changed (by marking areas).
         */
        void rebuildChangedTiles(const std::vector<int>& removedMarkedAreaIDs, const std::vector<int>& removedOffMeshConnections);

        /**
         * @brief Adds an agent to the navigation.
         * @param parameters    The parameters to initialize the agent with.
         * @return  True if everything worked out, false otherwise.
         */
        bool addAgent(Ref<DetourCrowdAgent> agent, Ref<DetourCrowdAgentParameters> parameters, bool main = true);

        /**
         * @brief Remove the passed crowd agent.
         */
        void removeAgent(dtCrowdAgent* agent);

        /**
         * @brief Adds the passed obstacle to this navmesh.
         */
        void addObstacle(Ref<DetourObstacle> obstacle);

        /**
         * @brief   Updates the internal detour classes, agents, etc.
         *          Called from the navigation thread!
         */
        void update(float timeDeltaSeconds);

        /**
         * @brief Create a debug representation of this navigation mesh and attach it to the MeshInstance as a mesh.
         */
        void createDebugMesh(GodotDetourDebugDraw* debugDrawer, bool drawCacheBounds);

        /**
         * @brief Get this navigation mesh's crowd.
         */
        dtCrowd* getCrowd();

        /**
         * @brief getActorFitFactor Returns how well an actor with the passed stats would fit this navmesh's crowd.
         * @return -1.0f if the actor does not fit at all (radius or height too big), otherwise a positive value - the SMALLER, the better the fit.
         */
        float getActorFitFactor(float agentRadius, float agentHeight);

    private:
        /**
         * @brief Initializes this mesh's crowd.
         * @return True if everything worked out.
         */
        bool initializeCrowd();

        /**
         * @brief Rasterize all layers of this tile, preparing them to be in the tile cache.
         */
        int rasterizeTileLayers(const int tileX, const int tileZ, const rcConfig& cfg, TileCacheData* tiles, const int maxTiles);

        /**
         * @brief Draws the tiles using the passed debug drawer.
         */
        void debugDrawTiles(GodotDetourDebugDraw* debugDrawer);

        /**
         * @brief Draws the obstacles using the passed debug drawer.
         */
        void debugDrawObstacles(GodotDetourDebugDraw* debugDrawer);

    private:
        RecastContext*          _recastContext;
        rcConfig*               _rcConfig;
        dtTileCache*            _tileCache;
        dtNavMesh*              _navMesh;
        dtNavMeshQuery*         _navQuery;
        dtCrowd*                _crowd;
        LinearAllocator*        _allocator;
        FastLZCompressor*       _compressor;
        MeshProcess*            _meshProcess;
        DetourInputGeometry*    _inputGeom;

        float   _maxAgentSlope;
        float   _maxAgentHeight;
        float   _maxAgentClimb;
        float   _maxAgentRadius;

        int     _maxAgents;
        int     _maxObstacles;
        int     _maxLayers;
        int     _navQueryMaxNodes;

        Vector2 _cellSize;
        int     _tileSize;
        int     _layersPerTile;

        int     _navMeshIndex;

        std::map<int, ChangedTileLayers> _affectedTilesByVolume;
        std::map<int, ChangedTileLayers> _affectedTilesByConnection;
    };


    // INLINES
    inline dtCrowd*
    DetourNavigationMesh::getCrowd()
    {
        return _crowd;
    }
}

#endif // DETOURNAVIGATIONMESH_H
