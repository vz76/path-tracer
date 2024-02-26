SET(expected_glm_dir ${CMAKE_SOURCE_DIR}/third-party/glm)
IF (NOT EXISTS ${expected_glm_dir}/copying.txt)
	EXECUTE_PROCESS(COMMAND git clone -b 0.9.9.7 --single-branch --depth 1 https://github.com/g-truc/glm.git ${expected_glm_dir})
ENDIF()
IF (NOT EXISTS ${expected_glm_dir}/glm/gtx/extended_min_max.hpp)
	EXECUTE_PROCESS(COMMAND rm -rf ${expected_glm_dir})
	EXECUTE_PROCESS(COMMAND git clone -b 0.9.9.7 --single-branch --depth 1 https://github.com/g-truc/glm.git ${expected_glm_dir})
ENDIF()
INCLUDE_DIRECTORIES(BEFORE SYSTEM ${expected_glm_dir})
		message(STATUS "Using bundled glm at ${expected_glm_dir}")

add_definitions(-DGLM_ENABLE_EXPERIMENTAL -DGLM_FORCE_SIZE_FUNC=1 -DGLM_FORCE_RADIANS=1)

# vim: tw=78
