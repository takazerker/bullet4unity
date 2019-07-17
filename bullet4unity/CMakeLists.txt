file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

include_directories(${BULLET_INCLUDE_DIR})

add_library(bullet4unity SHARED ${SOURCE_FILES} ${HEADER_FILES})
add_library(bullet4unity_static STATIC ${SOURCE_FILES} ${HEADER_FILES})

if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4190")
	
	target_compile_options(
		bullet4unity PRIVATE 
		"$<$<CONFIG:Debug>:/MTd>"
		"$<$<CONFIG:RelWithDebInfo>:/MT>"
		"$<$<CONFIG:Release>:/MT>"
		"$<$<CONFIG:MinSizeRel>:/MT>"
	)

	target_compile_options(
		bullet4unity_static PRIVATE 
		"$<$<CONFIG:Debug>:/MTd>"
		"$<$<CONFIG:RelWithDebInfo>:/MT>"
		"$<$<CONFIG:Release>:/MT>"
		"$<$<CONFIG:MinSizeRel>:/MT>"
	)
endif()

target_link_libraries(bullet4unity BulletInverseDynamics BulletSoftBody BulletCollision BulletDynamics LinearMath)

add_dependencies(bullet4unity_static BulletInverseDynamics BulletSoftBody BulletCollision BulletDynamics LinearMath)

