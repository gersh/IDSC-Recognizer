cmake_minimum_required(VERSION 2.8)
PROJECT( plantid)
FIND_PACKAGE( OpenCV REQUIRED )
ADD_EXECUTABLE( idsc src/idsc.cxx src/shape_descriptor.cxx src/reference_point.cxx)
TARGET_LINK_LIBRARIES( idsc ${OpenCV_LIBS} )

