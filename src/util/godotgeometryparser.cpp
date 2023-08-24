#include "godotgeometryparser.h"
#include <godot_cpp/variant/builtin_types.hpp>
#include <godot_cpp/classes/csg_shape3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/primitive_mesh.hpp>
#include <godot_cpp/classes/static_body3d.hpp>
#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/capsule_mesh.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/sphere_shape3d.hpp>
#include <godot_cpp/classes/sphere_mesh.hpp>
#include <godot_cpp/classes/geometry3d.hpp>
#include <godot_cpp/classes/grid_map.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>

using namespace godot;

GodotGeometryParser::GodotGeometryParser()
{

}

GodotGeometryParser::~GodotGeometryParser()
{
}

void
GodotGeometryParser::getNodeVerticesAndIndices(godot::MeshInstance3D* meshInstance, std::vector<float>& outVertices, std::vector<int>& outIndices)
{
    parseGeometry(meshInstance, outVertices, outIndices);
}

void
GodotGeometryParser::addMesh(const Ref<ArrayMesh>& p_mesh, const Transform3D& p_xform, std::vector<float>& p_vertices, std::vector<int>& p_indices)
{
    int current_vertex_count = 0;

    // Reserve indices & vertices to avoid dynamic allocation
    size_t vertexCount = 0;
    size_t faceCount = 0;
    for (int i = 0; i < p_mesh->get_surface_count(); i++) {
        if (p_mesh->surface_get_primitive_type(i) != Mesh::PRIMITIVE_TRIANGLES)
        {
            WARN_PRINT(String("Surface {0} not of type PRIMITIVE_TRIANGLES. Ignored.").format(Array::make(i)));
            continue;
        }

        int index_count = 0;
        if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX)
        {
            index_count = p_mesh->surface_get_array_index_len(i);
        }
        else {
            index_count = p_mesh->surface_get_array_len(i);
        }

        ERR_CONTINUE((index_count == 0 || (index_count % 3) != 0));

        int face_count = index_count / 3;

        Array a = p_mesh->surface_get_arrays(i);

        PackedVector3Array mesh_vertices = a[Mesh::ARRAY_VERTEX];
        vertexCount += mesh_vertices.size();
        faceCount += face_count;
    }
    p_vertices.reserve(vertexCount * 3);
    p_indices.reserve(faceCount * 3);

    for (int i = 0; i < p_mesh->get_surface_count(); i++) {
        current_vertex_count = p_vertices.size() / 3;

        if (p_mesh->surface_get_primitive_type(i) != Mesh::PRIMITIVE_TRIANGLES)
        {
            WARN_PRINT(String("Surface {0} not of type PRIMITIVE_TRIANGLES. Ignored.").format(Array::make(i)));
            continue;
        }

        int index_count = 0;
        if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX)
        {
            index_count = p_mesh->surface_get_array_index_len(i);
        }
        else {
            index_count = p_mesh->surface_get_array_len(i);
        }

        ERR_CONTINUE((index_count == 0 || (index_count % 3) != 0));

        int face_count = index_count / 3;

        Array a = p_mesh->surface_get_arrays(i);

        PackedVector3Array mesh_vertices = a[Mesh::ARRAY_VERTEX];
//        PackedVector3Array vr = mesh_vertices.read();

        if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX)
        {

            PackedInt32Array mesh_indices = a[Mesh::ARRAY_INDEX];
//            PackedIntArray ir = mesh_indices.read();

            for (int j = 0; j < mesh_vertices.size(); j++)
            {
                addVertex(p_xform.xform(mesh_vertices[j]), p_vertices);
            }

            for (int j = 0; j < face_count; j++)
            {
                // CCW
                p_indices.push_back(current_vertex_count + (mesh_indices[j * 3 + 0]));
                p_indices.push_back(current_vertex_count + (mesh_indices[j * 3 + 2]));
                p_indices.push_back(current_vertex_count + (mesh_indices[j * 3 + 1]));
            }
        }
        else {
            face_count = mesh_vertices.size() / 3;
            for (int j = 0; j < face_count; j++)
            {
                addVertex(p_xform.xform(mesh_vertices[j * 3 + 0]), p_vertices);
                addVertex(p_xform.xform(mesh_vertices[j * 3 + 2]), p_vertices);
                addVertex(p_xform.xform(mesh_vertices[j * 3 + 1]), p_vertices);

                p_indices.push_back(current_vertex_count + (j * 3 + 0));
                p_indices.push_back(current_vertex_count + (j * 3 + 1));
                p_indices.push_back(current_vertex_count + (j * 3 + 2));
            }
        }
    }
}

void
GodotGeometryParser::parseGeometry(godot::MeshInstance3D* meshInstance, std::vector<float> &p_vertices, std::vector<int> &p_indices)
{
    Ref<ArrayMesh> mesh = meshInstance->get_mesh();
    if (mesh.is_valid())
    {
        addMesh(mesh, meshInstance->get_transform(), p_vertices, p_indices);
    }
    else {
        // Not an array mesh, check if we have a primitive mesh to convert
        Ref<PrimitiveMesh> primitive_mesh = meshInstance->get_mesh();
        if (primitive_mesh.is_valid())
        {
            Ref<ArrayMesh> arr_mesh = new ArrayMesh();
            arr_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, primitive_mesh->get_mesh_arrays());
            addMesh(arr_mesh, meshInstance->get_transform(), p_vertices, p_indices);
        }
        else {
            ERR_PRINT(String("Mesh not an ArrayMesh or PrimitiveMesh"));
        }
    }
}
