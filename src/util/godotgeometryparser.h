#ifndef GDGEOMETRYPARSER_H
#define GDGEOMETRYPARSER_H

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <vector>

/**
 * @brief Will parse a passed MeshInstance
 */
class GodotGeometryParser {
    public:
    GodotGeometryParser();
    ~GodotGeometryParser();

    void getNodeVerticesAndIndices(godot::MeshInstance3D* meshInstance, std::vector<float>& outVertices, std::vector<int>& outIndices);

private:
    void addVertex(const godot::Vector3& p_vec3, std::vector<float>& _verticies);
    void addMesh(const godot::Ref<godot::ArrayMesh>& p_mesh, const godot::Transform3D& p_xform, std::vector<float>& p_vertices, std::vector<int>& p_indices);
    void parseGeometry(godot::MeshInstance3D* meshInstance, std::vector<float>& p_vertices, std::vector<int>& p_indices);
};

inline void
GodotGeometryParser::addVertex(const godot::Vector3& p_vec3, std::vector<float>& p_vertices)
{
    p_vertices.emplace_back(p_vec3.x);
    p_vertices.emplace_back(p_vec3.y);
    p_vertices.emplace_back(p_vec3.z);
}
#endif // GDGEOMETRYPARSER_H
