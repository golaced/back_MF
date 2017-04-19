TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/px4/src/flow_opencv.cpp \
    src/px4/src/trackFeatures.cpp \
    src/aruco.cpp \
    src/AttitudePosition.cpp \
    src/MarkerWorldCoornidate.cpp \
    src/inifile.cpp

include(deployment.pri)
qtcAddDeployment()

INCLUDEPATH += /usr/include/opencv\
               /home/pi/QT/MF/src/px4/include
LIBS    += -lopencv_highgui \
            -lopencv_photo \
            -lopencv_calib3d \
            -lopencv_imgproc \
            -lopencv_stitching \
            -lopencv_contrib \
            -lopencv_legacy \
            -lopencv_superres \
            -lopencv_core \
            -lopencv_ml \
            -lopencv_video \
            -lopencv_features2d \
            -lopencv_videostab \
            -lopencv_flann \
            -lopencv_objdetect \
            -lopencv_gpu \
            -lopencv_ocl \
            -laruco \
            -lpthread\
            -lserial\
            -lwiringPi\


HEADERS += \
    src/px4/include/flow_opencv.hpp \
    src/px4/include/optical_flow.hpp \
    src/px4/include/trackFeatures.h \
    src/AttitudePosition.h \
    src/MarkerWorldCoornidate.h \
    src/my_serial.h \
    src/inifile.h
