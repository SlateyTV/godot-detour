#include "godotdetour.h"

#include <gdextension_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/global_constants.hpp>

using namespace godot;

void initialize_godot_detour_module(ModuleInitializationLevel p_level)
{
    if (p_level == MODULE_INITIALIZATION_LEVEL_SCENE) {
        ClassDB::register_class<DetourNavigationParameters>();
        ClassDB::register_class<DetourNavigation>();
        ClassDB::register_class<DetourNavigationMeshParameters>();
        ClassDB::register_class<DetourNavigationMesh>();
        ClassDB::register_class<DetourCrowdAgentParameters>();
        ClassDB::register_class<DetourCrowdAgent>();
        ClassDB::register_class<DetourObstacle>();
    }

    if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
    }
}

void uninitialize_godot_detour_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" GDExtensionBool GDE_EXPORT godot_detour_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library,
                                                        GDExtensionInitialization *r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_godot_detour_module);
    init_obj.register_terminator(uninitialize_godot_detour_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
}