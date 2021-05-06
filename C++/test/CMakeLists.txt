add_executable( test_bsp test.cpp)
target_link_libraries( test_bsp bspline)

add_executable( exp exp.cpp)
target_link_libraries( exp bspline)
