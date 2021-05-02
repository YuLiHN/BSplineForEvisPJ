add_library( bspline SHARED
    lie_bspline.cpp
)

target_link_libraries( bspline
    ${THIRD_PARTY_LIBS}
    ${Sophus_LIBRARIES}
)
