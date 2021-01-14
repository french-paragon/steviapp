
set(STEVIAPP_LIB_PREFIX StereoVisionApp)

macro(add_stereovisionapp_libdirectory directory)
    add_subdirectory(${directory})
    set(${lib_var_name} ${lib_name} PARENT_SCOPE)
endmacro()

macro(define_stereovisionapp_lib varname name)
    set(lib_var_name ${varname}_LIB PARENT_SCOPE)
    set(lib_name ${STEVIAPP_LIB_PREFIX}${name} PARENT_SCOPE)
    set(${varname}_LIB ${STEVIAPP_LIB_PREFIX}${name} PARENT_SCOPE)
    set(${varname}_LIB ${STEVIAPP_LIB_PREFIX}${name})
endmacro()

add_stereovisionapp_libdirectory(geometry)
add_stereovisionapp_libdirectory(datablocks)
add_stereovisionapp_libdirectory(processing)
add_stereovisionapp_libdirectory(gui)
add_stereovisionapp_libdirectory(sparsesolver)
add_stereovisionapp_libdirectory(vision)
