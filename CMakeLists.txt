# ESP-IDF component definition. The host build (tests and examples) is separate:
# see tests/host/CMakeLists.txt.
# SRC_DIRS lists each source directory explicitly (the search is not recursive).
idf_component_register(
    SRC_DIRS lib lib/core lib/serial lib/platform lib/families/stseries
    INCLUDE_DIRS include
    REQUIRES log driver)
