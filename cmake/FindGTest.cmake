# ============================================================================
# Google Test and Google Mock Configuration
# ============================================================================
#
# This module configures Google Test (GTest) and Google Mock (GMock) for use
# in the project. Both frameworks are fetched from the official repository
# and made available as CMake targets.
#
# Available targets after including this module:
#   - GTest::gtest       - Google Test framework
#   - GTest::gtest_main  - Google Test main function
#   - GTest::gmock       - Google Mock framework
#   - GTest::gmock_main  - Google Mock main function
#
# ============================================================================

include(FetchContent)

# Define where to get Google Test and Google Mock from
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
    GIT_SHALLOW TRUE  # Only fetch the latest commit for faster downloads
)

# Show download progress (useful for transparency)
set(FETCHCONTENT_QUIET OFF)

# Configure Google Test/Mock build options
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)  # Use shared runtime on Windows
set(gmock_build_tests OFF CACHE BOOL "" FORCE)      # Don't build GMock's own tests
set(gtest_build_tests OFF CACHE BOOL "" FORCE)      # Don't build GTest's own tests
set(gtest_build_samples OFF CACHE BOOL "" FORCE)    # Don't build GTest samples

# Disable installation of Google Test/Mock (we only need it for building)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(INSTALL_GMOCK OFF CACHE BOOL "" FORCE)

# Actually retrieve and make the dependency available
FetchContent_MakeAvailable(googletest)

# Create aliases for consistency with find_package result
if(NOT TARGET GTest::gtest)
    add_library(GTest::gtest ALIAS gtest)
endif()

if(NOT TARGET GTest::gtest_main)
    add_library(GTest::gtest_main ALIAS gtest_main)
endif()

if(NOT TARGET GTest::gmock)
    add_library(GTest::gmock ALIAS gmock)
endif()

if(NOT TARGET GTest::gmock_main)
    add_library(GTest::gmock_main ALIAS gmock_main)
endif()

# Set global properties for better integration
set_target_properties(gtest gtest_main gmock gmock_main PROPERTIES
    FOLDER "External/GoogleTest"
)
