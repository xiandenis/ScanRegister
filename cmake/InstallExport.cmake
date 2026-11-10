# Install headers + export CMake package for find_package(real_time_registration).
#
# REAL_TIME_REGISTRATION_INSTALL_SCOPE（根 CMakeLists 设置）:
#   party (default) — 头文件：registration_type + real_time_registration + logging
#                     （另装 normal_estimate.hpp）。
#                     库：real_time_manager 链接闭包（含 libregistration_type.so / liblogging.so）。
#   full            — 全部库目标；头文件为全量减去内部/第三方式叶子模块
#                     （pmc、xenium、matcher_tsl/points/kdtree/hnswlib）。
#   lidarstudio     — LidarStudio 产品头白名单（+ fine 编译传递头）；
#                     库：白名单根目标链接闭包（更深依赖只装 .lib/.dll，不装头）。

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

string(TOLOWER "${REAL_TIME_REGISTRATION_INSTALL_SCOPE}" _RTR_SCOPE)
if(NOT _RTR_SCOPE STREQUAL "party" AND NOT _RTR_SCOPE STREQUAL "full"
   AND NOT _RTR_SCOPE STREQUAL "lidarstudio")
  message(FATAL_ERROR
    "REAL_TIME_REGISTRATION_INSTALL_SCOPE must be 'party', 'full', or 'lidarstudio' (got '${REAL_TIME_REGISTRATION_INSTALL_SCOPE}')")
endif()

# ---- Full catalog (historical install set) ---------------------------------
set(_RTR_LIB_TARGETS_FULL
  basic_icp
  common
  depth_map_tool
  fast_gicp
  fastreg_filter
  file_system
  fine_registration
  global_matrix_file
  graph
  logging
  matcher_hnswlib
  matcher_kdtree
  matcher_points
  matcher_tsl
  octree
  overlap_estimation
  pmc
  point_cloud_io
  pose_graph_tool
  pose_graph_opt
  real_time_manager
  real_time_registration
  registration_initializer
  registration_report
  registration_type
  resolution_estimate
  robin
  robust_global_reg
  rough_global_reg
  robust_matcher
  small_gicp
  teaser
  xenium
)

set(_RTR_HEADER_MODULES_FULL
  basic_icp
  common
  depth_map_tool
  fast_gicp
  fastreg_filter
  file_system
  fine_registration
  global_matrix_file
  graph
  logging
  matcher_hnswlib
  matcher_kdtree
  matcher_points
  matcher_tsl
  octree
  overlap_estimation
  pmc
  point_cloud_io
  pose_graph_tool
  pose_graph_opt
  real_time_registration
  registration_initializer
  registration_report
  registration_type
  resolution_estimate
  robin
  robust_global_reg
  rough_global_reg
  robust_matcher
  small_gicp
  teaser
  xenium
  cmake
)

# Party：对外头文件模块（与库一并交付）
set(_RTR_HEADER_MODULES_PARTY
  registration_type
  real_time_registration
  logging
)

# Party：必须存在的 SHARED 库目标（装 .so/.dll + 对应头目录）
set(_RTR_PARTY_SHARED_TARGETS
  logging
  registration_type
)

# Party extras: Host ICP-LOOP prereq — pose_graph_opt is not in real_time_manager link closure.
set(_RTR_PARTY_EXTRA_TARGETS
  pose_graph_opt
)
set(_RTR_PARTY_EXTRA_HEADERS
  pose_graph_opt
)

# LidarStudio：产品面 + fine 编译传递头（整目录装 *.hpp/*.h）
set(_RTR_HEADER_MODULES_LIDARSTUDIO
  cloud_chunk
  cloud_denoise
  cloud_fusion
  cloud_sample
  cloud_smooth
  logging
  octree
  overlap_estimation
  registration_type
  rough_global_reg
  fine_registration
  auto_registration
  file_system
  point_cloud_io
  common
  registration_report
  basic_icp
)

# LidarStudio：链接闭包根目标（含传递库 basic_icp / registration_report / common）
set(_RTR_LIDARSTUDIO_ROOT_TARGETS
  cloud_chunk
  cloud_denoise
  cloud_fusion
  cloud_sample
  cloud_smooth
  logging
  octree
  overlap_estimation
  registration_type
  rough_global_reg
  fine_registration
  auto_registration
  file_system
  point_cloud_io
  common
  registration_report
  basic_icp
)

# Collect in-tree link closure (skip IMPORTED / genex / absolute paths / -l flags).
function(rtr_collect_link_closure root_target out_var)
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

if(_RTR_SCOPE STREQUAL "party")
  rtr_collect_link_closure(real_time_manager _RTR_LIB_TARGETS)
  # Party consumers link real_time_manager；不导出伞形 INTERFACE。
  list(REMOVE_ITEM _RTR_LIB_TARGETS real_time_registration)
  # 显式纳入 SHARED 模块（logging / registration_type）：头+库都装。
  foreach(_shared IN LISTS _RTR_PARTY_SHARED_TARGETS)
    if(TARGET ${_shared})
      list(APPEND _RTR_LIB_TARGETS ${_shared})
    endif()
  endforeach()
  foreach(_extra IN LISTS _RTR_PARTY_EXTRA_TARGETS)
    if(TARGET ${_extra})
      list(APPEND _RTR_LIB_TARGETS ${_extra})
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _RTR_LIB_TARGETS)
  set(_RTR_HEADER_MODULES ${_RTR_HEADER_MODULES_PARTY})
  foreach(_extra IN LISTS _RTR_PARTY_EXTRA_HEADERS)
    list(APPEND _RTR_HEADER_MODULES ${_extra})
  endforeach()
  list(REMOVE_DUPLICATES _RTR_HEADER_MODULES)
  message(STATUS "Install scope: party")
  message(STATUS "  headers : registration_type + real_time_registration + logging (+ normal_estimate.hpp)")
  message(STATUS "  shared  : liblogging / libregistration_type (+ real_time_manager static closure)")
elseif(_RTR_SCOPE STREQUAL "lidarstudio")
  set(_RTR_LIB_TARGETS "")
  foreach(_root IN LISTS _RTR_LIDARSTUDIO_ROOT_TARGETS)
    if(TARGET ${_root})
      rtr_collect_link_closure(${_root} _rtr_closure)
      list(APPEND _RTR_LIB_TARGETS ${_rtr_closure})
    else()
      message(WARNING "lidarstudio install: root target '${_root}' not found; skipped")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES _RTR_LIB_TARGETS)
  set(_RTR_HEADER_MODULES ${_RTR_HEADER_MODULES_LIDARSTUDIO})
  message(STATUS "Install scope: lidarstudio")
  message(STATUS "  headers : ${_RTR_HEADER_MODULES}")
  message(STATUS "  libs    : link closure of lidarstudio roots (${_RTR_LIB_TARGETS})")
else()
  set(_RTR_LIB_TARGETS ${_RTR_LIB_TARGETS_FULL})
  set(_RTR_HEADER_MODULES ${_RTR_HEADER_MODULES_FULL})
  # 内部/叶子头不对外安装（库目标仍导出，供静态链接闭包使用）。
  set(_RTR_HEADER_EXCLUDE_FULL
    pmc
    xenium
    matcher_tsl
    matcher_points
    matcher_kdtree
    matcher_hnswlib
  )
  list(REMOVE_ITEM _RTR_HEADER_MODULES ${_RTR_HEADER_EXCLUDE_FULL})
  message(STATUS "Install scope: full (all lib targets; headers minus pmc/xenium/matcher_*)")
  message(STATUS "  header exclude: ${_RTR_HEADER_EXCLUDE_FULL}")
endif()

# Export only synthetic INTERFACE helpers created in this project's Dependencies.cmake.
set(_RTR_HELPER_TARGETS)
foreach(_helper IN ITEMS real_time_registration_spdlog opencv_world)
  if(TARGET ${_helper})
    get_target_property(_imported ${_helper} IMPORTED)
    get_target_property(_aliased ${_helper} ALIASED_TARGET)
    if(_imported OR _aliased)
      # External / alias — resolved by find_dependency on the consumer side.
    else()
      list(APPEND _RTR_HELPER_TARGETS ${_helper})
    endif()
  endif()
endforeach()

set(_RTR_EXPORT_TARGETS)
foreach(_t IN LISTS _RTR_LIB_TARGETS _RTR_HELPER_TARGETS)
  if(TARGET ${_t})
    list(APPEND _RTR_EXPORT_TARGETS ${_t})
  endif()
endforeach()
list(REMOVE_DUPLICATES _RTR_EXPORT_TARGETS)

foreach(_t IN LISTS _RTR_LIB_TARGETS)
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

install(TARGETS ${_RTR_EXPORT_TARGETS}
  EXPORT real_time_registrationTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

foreach(_mod IN LISTS _RTR_HEADER_MODULES)
  if(IS_DIRECTORY "${REAL_TIME_REGISTRATION_INCLUDE_ROOT}/${_mod}")
    install(DIRECTORY "${REAL_TIME_REGISTRATION_INCLUDE_ROOT}/${_mod}/"
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
endforeach()

# Party: Manager 公开头依赖 NormalEstimateMethod（不整目录安装 registration_initializer）。
if(_RTR_SCOPE STREQUAL "party")
  install(FILES
    "${REAL_TIME_REGISTRATION_INCLUDE_ROOT}/registration_initializer/normal_estimate.hpp"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/registration_initializer"
  )
endif()

install(EXPORT real_time_registrationTargets
  FILE real_time_registrationTargets.cmake
  NAMESPACE real_time_registration::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/real_time_registration"
)

configure_package_config_file(
  "${REAL_TIME_REGISTRATION_INCLUDE_ROOT}/cmake/real_time_registrationConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/real_time_registrationConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/real_time_registration"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/real_time_registrationConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/real_time_registrationConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/real_time_registrationConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/real_time_registration"
)

# Consumer-facing Session usage guide (copied into the install tree).
# party/full keep it; lidarstudio omits Session-facing README from package root noise.
set(_RTR_SESSION_README
  "${REAL_TIME_REGISTRATION_INCLUDE_ROOT}/real_time_registration/README.md")
if(EXISTS "${_RTR_SESSION_README}" AND NOT _RTR_SCOPE STREQUAL "lidarstudio")
  install(FILES "${_RTR_SESSION_README}"
    DESTINATION "${CMAKE_INSTALL_DOCDIR}"
  )
  install(FILES "${_RTR_SESSION_README}"
    DESTINATION "."
    RENAME "RealTimeRegistrationSession_README.md"
  )
endif()

# Convenience non-namespaced aliases for consumers that link `real_time_manager` directly.
# Provided via Config.cmake after including the exported targets.
