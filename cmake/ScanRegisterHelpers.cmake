# Shared helpers expected by module CMakeLists.txt files.

set(SCAN_REGISTER_COMPAT_HEADER
  "${CMAKE_CURRENT_LIST_DIR}/scan_register_compat.hpp"
  CACHE FILEPATH "Forced portability include for MSVC/POSIX"
)

function(scan_register_apply_compat target)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR "scan_register_apply_compat: target '${target}' does not exist")
  endif()

  if(MSVC)
    target_compile_options("${target}" PRIVATE "/FI${SCAN_REGISTER_COMPAT_HEADER}")
  else()
    target_compile_options("${target}" PRIVATE "-include${SCAN_REGISTER_COMPAT_HEADER}")
  endif()
endfunction()

function(scan_register_apply_openmp target)
  if(NOT TARGET "${target}")
    message(FATAL_ERROR "scan_register_apply_openmp: target '${target}' does not exist")
  endif()

  if(MSVC)
    # MSVC classic OpenMP requires signed loop indices; LLVM OpenMP matches GCC/Clang.
    target_compile_options("${target}" PUBLIC /openmp:llvm)
    if(TARGET OpenMP::OpenMP_CXX)
      target_link_libraries("${target}" PUBLIC OpenMP::OpenMP_CXX)
    endif()
  elseif(TARGET OpenMP::OpenMP_CXX)
    target_link_libraries("${target}" PUBLIC OpenMP::OpenMP_CXX)
  elseif(OpenMP_CXX_FOUND)
    target_compile_options("${target}" PUBLIC ${OpenMP_CXX_FLAGS})
    if(OpenMP_CXX_LIBRARIES)
      target_link_libraries("${target}" PUBLIC ${OpenMP_CXX_LIBRARIES})
    endif()
  else()
    message(WARNING "OpenMP not found; target '${target}' will build without OpenMP")
  endif()
endfunction()
