# ============================================================================
# Test Configuration with Google Test and Google Mock
# ============================================================================
#
# This CMakeLists.txt configures unit tests for the project using:
# - Google Test (GTest) for basic testing framework
# - Google Mock (GMock) for mocking and advanced testing patterns
#
# Each .cpp file in this directory is treated as a separate test executable.
# This allows for:
# - Isolated test execution
# - Better debugging (you can run individual test files)
# - Parallel test execution
# - Clear separation of test concerns
#
# ============================================================================

# Load Google Test and Google Mock configuration
include(${CMAKE_SOURCE_DIR}/cmake/FindGTest.cmake)  # Custom module to find and configure GTest/GMock
include(GoogleTest)  # Built-in CMake module for GTest integration

# Find all test files in the current directory
file(GLOB TEST_SOURCES "*.cpp")  # Collects all C++ files into TEST_SOURCES variable

# Display the found test files for debugging
message(STATUS "Found test files: ${TEST_SOURCES}")

# Process each test file individually
foreach(test_source ${TEST_SOURCES})
    # Extract the base name without extension to use as the test target name
    get_filename_component(test_name ${test_source} NAME_WE)
    
    message(STATUS "Configuring test: ${test_name}")
    
    # Create an executable for this test
    add_executable(${test_name} ${test_source})
    
    # Link necessary libraries to the test
    target_link_libraries(${test_name}
        PRIVATE           # These dependencies are only used internally by the test
        GTest::gtest      # Google Test framework
        GTest::gtest_main # Google Test main entry point
        GTest::gmock      # Google Mock framework
        GTest::gmock_main # Google Mock main entry point (includes gtest_main)
        ${PROJECT_NAME}::${PROJECT_NAME}  # Link to the main project library
    )
    
    # Set C++20 standard for tests (match the main project)
    target_compile_features(${test_name} PRIVATE cxx_std_20)
    
    # Apply warning configuration from elsewhere in the project
    target_compile_warnings(${test_name} PRIVATE)

    # Enable sanitizers for this test if requested at configuration time
    if(ENABLE_SANITIZERS)
        target_enable_sanitizers(${test_name})
    endif()
    
    # Enable code coverage instrumentation if requested
    if(ENABLE_COVERAGE)
        target_compile_options(${test_name} PRIVATE --coverage)
        target_link_libraries(${test_name} PRIVATE --coverage)
    endif()
    
    # Register with CTest for test discovery and running
    # This allows running tests with 'ctest' or 'cmake --build . --target test'
    gtest_discover_tests(${test_name}
        TIMEOUT 30                    # Increase timeout for more complex tests
        DISCOVERY_MODE PRE_TEST       # Discover tests at test time (more reliable)
        PROPERTIES 
            TIMEOUT 30                # Set individual test timeout
            LABELS "unit"             # Label for grouping tests
    )
endforeach()

# Add a custom target to run only unit tests
add_custom_target(run_unit_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure --label-regex "unit"
    DEPENDS ${TEST_SOURCES}
    COMMENT "Running unit tests"
)