find_package( g2o REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
list(APPEND THIRD_PART_LIBRARIES g2o_core g2o_stuff)