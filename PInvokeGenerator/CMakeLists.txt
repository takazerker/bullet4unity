
configure_file(PInvokeGenerator.csproj ${CMAKE_BINARY_DIR}/PInvokeGenerator/PInvokeGenerator.csproj)

include_external_msproject(PInvokeGenerator PInvokeGenerator/PInvokeGenerator.csproj)

add_custom_target(
	GeneratePInvoke
	dotnet ${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/PInvokeGenerator.dll ${CMAKE_SOURCE_DIR}/bullet4unity/bullet4unity.cpp
	depends PInvokeGenerator
)

