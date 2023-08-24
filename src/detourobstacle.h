#ifndef DETOUROBSTACLE_H
#define DETOUROBSTACLE_H

#include <map>
#include <vector>
#include <godot_cpp/classes/ref_counted.hpp>

class dtTileCache;

// Obstacle types
enum DetourObstacleType
{
    OBSTACLE_TYPE_INVALID = -1,
    OBSTACLE_TYPE_CYLINDER,
    OBSTACLE_TYPE_BOX,
    NUM_OBSTACLE_TYPES
};

namespace godot
{
    class FileAccess;

    /**
     * @brief A single obstacle. Can be moved around or destroyed.
     */
    class DetourObstacle : public RefCounted
    {
        GDCLASS(DetourObstacle, RefCounted);

    public:
        static void _bind_methods();

        /**
         * @brief Destructor.
         */
        DetourObstacle();

        /**
         * @brief Destructor.
         */
        ~DetourObstacle();

        /**
         * @brief Called when .new() is called in gdscript
         */
        void _init() {}

        /**
         * @brief Returns the type of this obstacle.
         */
        DetourObstacleType getType() const;

        /**
         * @brief Initialize the obstacle.
         */
        void initialize(DetourObstacleType type, const Vector3& position, const Vector3& dimensions, float rotationRad);

        /**
         * @brief Will save this obstacle's current state to the passed file.
         * @param targetFile The file to append data to.
         * @return True if everything worked out, false otherwise.
         */
        bool save(Ref<FileAccess> targetFile);

        /**
         * @brief Loads the obstacle from the file.
         * @param sourceFile The file to read data from.
         * @return True if everything worked out, false otherwise.
         */
        bool load(Ref<FileAccess> sourceFile);

        /**
         * @brief Create the obstacle using the passed tile cache. Will also remember the reference.
         */
        void createDetourObstacle(dtTileCache* cache);

        /**
         * @brief Pass the reference of this obstacle and the navmesh it is in.
         */
        void addReference(unsigned int ref, dtTileCache* cache);

        /**
         * @brief Move this obstacle to a new position.
         */
        void move(Vector3 position);

        /**
         * @brief Destroy this obstacle, removing it from all navmeshes.
         */
        void destroy();

        /**
         * @brief Returns true if this was destroyed.
         */
        bool isDestroyed();

    private:
        DetourObstacleType _type;

        Vector3 _position;
        Vector3 _dimensions; // In case of cylinder, x = radius, y = height, z = unused
        float   _rotationRad;
        bool    _destroyed;

        std::map<dtTileCache*, unsigned int> _references;

    public:
        void set_position(Vector3 position) {
            _position = position;
        }
        Vector3 get_position() const {
            return _position;
        }

        void set_dimensions(Vector3 dimensions) {
            _dimensions = dimensions;
        }
        Vector3 get_dimensions() const {
            return _dimensions;
        }
    };


    inline DetourObstacleType
    DetourObstacle::getType() const
    {
        return _type;
    }

    inline bool
    DetourObstacle::isDestroyed()
    {
        return _destroyed;
    }
}

#endif // DETOUROBSTACLE_H
