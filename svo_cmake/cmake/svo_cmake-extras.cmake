# Make the SVO cmake modules (e.g. SvoSetup.cmake) available to downstream packages.
list(INSERT CMAKE_MODULE_PATH 0 "${svo_cmake_DIR}/Modules")
