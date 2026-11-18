# CMake generated Testfile for 
# Source directory: /home/tylor/Projects/hearth-dsp
# Build directory: /home/tylor/Projects/hearth-dsp/build-asan
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(hdsp "/home/tylor/Projects/hearth-dsp/build-asan/hdsp_test")
set_tests_properties(hdsp PROPERTIES  _BACKTRACE_TRIPLES "/home/tylor/Projects/hearth-dsp/CMakeLists.txt;29;add_test;/home/tylor/Projects/hearth-dsp/CMakeLists.txt;0;")
add_test(hdsp_modules "/home/tylor/Projects/hearth-dsp/build-asan/hdsp_modules_test")
set_tests_properties(hdsp_modules PROPERTIES  _BACKTRACE_TRIPLES "/home/tylor/Projects/hearth-dsp/CMakeLists.txt;36;add_test;/home/tylor/Projects/hearth-dsp/CMakeLists.txt;0;")
