# Minimal CHOLMOD config exposing SuiteSparse::CHOLMOD for g2o on jammy
set(CHOLMOD_FOUND TRUE)
if(NOT TARGET SuiteSparse::CHOLMOD)
  add_library(SuiteSparse::CHOLMOD UNKNOWN IMPORTED)
  set_target_properties(SuiteSparse::CHOLMOD PROPERTIES
    IMPORTED_LOCATION /usr/lib/x86_64-linux-gnu/libcholmod.so
    INTERFACE_INCLUDE_DIRECTORIES /usr/include/suitesparse
    INTERFACE_LINK_LIBRARIES "/usr/lib/x86_64-linux-gnu/libamd.so;/usr/lib/x86_64-linux-gnu/libcolamd.so;/usr/lib/x86_64-linux-gnu/libcamd.so;/usr/lib/x86_64-linux-gnu/libccolamd.so;/usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so;/usr/lib/x86_64-linux-gnu/libblas.so;/usr/lib/x86_64-linux-gnu/liblapack.so"
  )
endif()
