# Install headers + export CMake package for find_package(scan_register).
#
# SCAN_REGISTER_INSTALL_SCOPE（根 CMakeLists 设置）:
#   party (default) — Session / 对外面：清单内头 + 根目标链接闭包（模块落地后填写）。
#   full            — 全量库目标与头模块清单（模块落地后填写）。
#   lidarstudio     — LidarStudio 产品白名单根目标链接闭包（模块落地后填写）。
#
# 已落地：common / file_system / logging / point_cloud_io /
#         robust_rough/* / robust_refine/* / robust_global_reg。

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

string(TOLOWER "${SCAN_REGISTER_INSTALL_SCOPE}" _SR_SCOPE)
if(NOT _SR_SCOPE STREQUAL "party" AND NOT _SR_SCOPE STREQUAL "full"
   AND NOT _SR_SCOPE STREQUAL "lidarstudio")
  message(FATAL_ERROR
    "SCAN_REGISTER_INSTALL_SCOPE must be 'party', 'full', or 'lidarstudio' (got '${SCAN_REGISTER_INSTALL_SCOPE}')")
endif()

# ---- Catalogs --------------------------------------------------------------
set(_SR_LIB_TARGETS_FULL
  common
  file_system
  logging
  scan_register
  xenium
  matcher_tsl
  matcher_points
  matcher_kdtree
  matcher_hnswlib
  pmc
  teaser
  robin
  robust_matcher
  basic_icp
  fast_gicp
  small_gicp
  genz_icp
  point_cloud_io
  robust_global_reg
)

set(_SR_HEADER_MODULES_FULL
  common
  file_system
  logging
  teaser
  robin
  robust_matcher
  basic_icp
  fast_gicp
  small_gicp
  genz_icp
  point_cloud_io
  robust_global_reg
)

# Party：对外头文件模块（与库一并交付）
set(_SR_HEADER_MODULES_PARTY
  common
  file_system
  logging
)

# Party：链接闭包根目标（伞目标拉起 common / file_system / logging）
set(_SR_PARTY_ROOT_TARGETS
  scan_register
)

# Party：必须存在的 SHARED 库目标（装 .so/.dll + 对应头目录）
set(_SR_PARTY_SHARED_TARGETS
  common
  file_system
  logging
)

# LidarStudio：产品面头白名单
set(_SR_HEADER_MODULES_LIDARSTUDIO
  common
  file_system
  logging
)

# LidarStudio：链接闭包根目标
set(_SR_LIDARSTUDIO_ROOT_TARGETS
  scan_register
  common
  file_system
  logging
)

# Collect in-tree link closure (skip IMPORTED / genex / absolute paths / -l flags).
function(scan_register_collect_link_closure root_target out_var)
  set(_seen "")
  set(_queue "${root_target}")
  while(_queue)
    list(GET _queue 0 _t)
    list(REMOVE_AT _queue 0)
    if("x${_t}" STREQUAL "x")
      # skip empty
    elseif(_t IN_LIST _seen)
      # already visited
    elseif(NOT TARGET ${_t})
      # not a target
    else()
      get_target_property(_imported ${_t} IMPORTED)
      if(_imported)
        # skip imported
      else()
        list(APPEND _seen ${_t})
        set(_libs "")
        get_target_property(_il ${_t} INTERFACE_LINK_LIBRARIES)
        get_target_property(_ll ${_t} LINK_LIBRARIES)
        if(_il)
          list(APPEND _libs ${_il})
        endif()
        if(_ll)
          list(APPEND _libs ${_ll})
        endif()
        foreach(_dep IN LISTS _libs)
          if(_dep AND NOT _dep MATCHES "^\\$<" AND NOT _dep MATCHES "^-" AND NOT _dep MATCHES "^/" AND TARGET ${_dep})
            list(APPEND _queue ${_dep})
          endif()
        endforeach()
      endif()
    endif()
  endwhile()
  set(${out_var} "${_seen}" PARENT_SCOPE)
endfunction()

if(_SR_SCOPE STREQUAL "party")
  set(_SR_LIB_TARGETS "")
  foreach(_root IN LISTS _SR_PARTY_ROOT_TARGETS)
    if(TARGET ${_root})
      scan_register_collect_link_closure(${_root} _sr_closure)
      list(APPEND _SR_LIB_TARGETS ${_sr_closure})
    endif()
  endforeach()
  foreach(_shared IN LISTS _SR_PARTY_SHARED_TARGETS)
    if(TARGET ${_shared})
      list(APPEND _SR_LIB_TARGETS ${_shared})
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _SR_LIB_TARGETS)
  set(_SR_HEADER_MODULES ${_SR_HEADER_MODULES_PARTY})
  message(STATUS "Install scope: party")
  message(STATUS "  headers : ${_SR_HEADER_MODULES}")
  message(STATUS "  libs    : ${_SR_LIB_TARGETS}")
elseif(_SR_SCOPE STREQUAL "lidarstudio")
  set(_SR_LIB_TARGETS "")
  foreach(_root IN LISTS _SR_LIDARSTUDIO_ROOT_TARGETS)
    if(TARGET ${_root})
      scan_register_collect_link_closure(${_root} _sr_closure)
      list(APPEND _SR_LIB_TARGETS ${_sr_closure})
    else()
      message(WARNING "lidarstudio install: root target '${_root}' not found; skipped")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _SR_LIB_TARGETS)
  set(_SR_HEADER_MODULES ${_SR_HEADER_MODULES_LIDARSTUDIO})
  message(STATUS "Install scope: lidarstudio")
  message(STATUS "  headers : ${_SR_HEADER_MODULES}")
  message(STATUS "  libs    : link closure of lidarstudio roots (${_SR_LIB_TARGETS})")
else()
  set(_SR_LIB_TARGETS ${_SR_LIB_TARGETS_FULL})
  set(_SR_HEADER_MODULES ${_SR_HEADER_MODULES_FULL})
  message(STATUS "Install scope: full")
  message(STATUS "  headers : ${_SR_HEADER_MODULES}")
  message(STATUS "  libs    : ${_SR_LIB_TARGETS}")
endif()

# Export only synthetic INTERFACE helpers created in this project's Dependencies.cmake.
set(_SR_HELPER_TARGETS)
foreach(_helper IN ITEMS scan_register_spdlog opencv_world)
  if(TARGET ${_helper})
    get_target_property(_imported ${_helper} IMPORTED)
    get_target_property(_aliased ${_helper} ALIASED_TARGET)
    if(_imported OR _aliased)
      # External / alias — resolved by find_dependency on the consumer side.
    else()
      list(APPEND _SR_HELPER_TARGETS ${_helper})
    endif()
  endif()
endforeach()

set(_SR_EXPORT_TARGETS)
foreach(_t IN LISTS _SR_LIB_TARGETS _SR_HELPER_TARGETS)
  if(TARGET ${_t})
    list(APPEND _SR_EXPORT_TARGETS ${_t})
  endif()
endforeach()
list(REMOVE_DUPLICATES _SR_EXPORT_TARGETS)

foreach(_t IN LISTS _SR_LIB_TARGETS)
  if(TARGET ${_t})
    get_target_property(_type ${_t} TYPE)
    if(_type STREQUAL "INTERFACE_LIBRARY")
      target_include_directories(${_t} INTERFACE
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
      )
    else()
      target_include_directories(${_t} PUBLIC
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
      )
    endif()
  endif()
endforeach()

if(_SR_EXPORT_TARGETS)
  install(TARGETS ${_SR_EXPORT_TARGETS}
    EXPORT scan_registerTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(EXPORT scan_registerTargets
    FILE scan_registerTargets.cmake
    NAMESPACE scan_register::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/scan_register"
  )
else()
  # Empty catalog: ship a stub Targets file so Config.cmake can include it.
  set(_SR_TARGETS_STUB "${CMAKE_CURRENT_BINARY_DIR}/scan_registerTargets.cmake")
  file(WRITE "${_SR_TARGETS_STUB}"
"# Generated stub: no scan_register library targets exported yet.
# Append modules to InstallExport.cmake catalogs when they land.
")
  install(FILES "${_SR_TARGETS_STUB}"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/scan_register"
  )
  message(STATUS "Install: no in-tree library targets yet; shipping stub Targets.cmake")
endif()

foreach(_mod IN LISTS _SR_HEADER_MODULES)
  set(_SR_HDR_SRC "")
  if(IS_DIRECTORY "${SCAN_REGISTER_INCLUDE_ROOT}/robust_refine/${_mod}")
    set(_SR_HDR_SRC "${SCAN_REGISTER_INCLUDE_ROOT}/robust_refine/${_mod}")
  elseif(IS_DIRECTORY "${SCAN_REGISTER_INCLUDE_ROOT}/robust_rough/${_mod}")
    set(_SR_HDR_SRC "${SCAN_REGISTER_INCLUDE_ROOT}/robust_rough/${_mod}")
  elseif(IS_DIRECTORY "${SCAN_REGISTER_INCLUDE_ROOT}/${_mod}")
    set(_SR_HDR_SRC "${SCAN_REGISTER_INCLUDE_ROOT}/${_mod}")
  endif()
  if(_SR_HDR_SRC)
    if(_mod STREQUAL "matcher_hnswlib")
      # Public includes are <hnswlib/...>, not <matcher_hnswlib/...>.
      if(IS_DIRECTORY "${_SR_HDR_SRC}/hnswlib")
        install(DIRECTORY "${_SR_HDR_SRC}/hnswlib/"
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/hnswlib"
          FILES_MATCHING
            PATTERN "*.hpp"
            PATTERN "*.h"
            PATTERN "*.hxx"
            PATTERN "*.inl"
        )
      endif()
    else()
      install(DIRECTORY "${_SR_HDR_SRC}/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${_mod}"
        FILES_MATCHING
          PATTERN "*.hpp"
          PATTERN "*.h"
          PATTERN "*.hxx"
          PATTERN "*.inl"
          PATTERN "CMakeLists.txt" EXCLUDE
          PATTERN "*test*" EXCLUDE
          PATTERN "*_test.*" EXCLUDE
      )
    endif()
  endif()
endforeach()

configure_package_config_file(
  "${SCAN_REGISTER_INCLUDE_ROOT}/cmake/scan_registerConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/scan_registerConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/scan_register"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/scan_registerConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/scan_registerConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/scan_registerConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/scan_register"
)
