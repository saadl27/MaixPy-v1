#include "obj.h" // Include your necessary headers

// Define your function
STATIC mp_obj_t hello() {
    mp_printf(&mp_plat_print, "hello from my_lib");
    return mp_const_none;
}

// Define the constant function object
MP_DEFINE_CONST_FUN_OBJ_0(my_lib_func_hello_obj, hello);

// Global definitions table
STATIC const mp_map_elem_t my_lib_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_hello), (mp_obj_t)&my_lib_func_hello_obj },
};

// Define the global dictionary for your module
STATIC MP_DEFINE_CONST_DICT(my_lib_globals_dict, my_lib_globals_table);

// Module definition structure
const mp_obj_module_t my_lib_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&my_lib_globals_dict,
};

// Register your module
MP_REGISTER_MODULE(MP_QSTR_my_lib, my_lib_module, MODULE_MY_LIB_ENABLED);




