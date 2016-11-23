TEMPLATE = app

QT += qml quick concurrent 3dcore 3drender widgets multimedia

CONFIG += c++11

SOURCES += main.cpp \
    synchloop.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Default rules for deployment.
include(deployment.pri)

HEADERS += \
    synchloop.h

# Opencv dep
LIBS += -L/usr/local/lib -lopencv_calib3d -lopencv_highgui -lopencv_imgproc -lopencv_core -lopencv_video
INCLUDEPATH += /usr/local/include/opencv2/

# ArToolkit dep
LIBS += -L/home/arzu/ARToolKit5/lib/ -Wl,-whole-archive -lAR -lARICP -lARMulti -lAR2 -lARUtil -Wl,-no-whole-archive -ljpeg
INCLUDEPATH += /home/arzu/ARToolKit5/include

#To run LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/chili/opencv-2.4.13/build-linux/install/lib:/home/chili/artoolkit5/lib
#--video=/home/chili/QTProjects/EyeTrackingExtractAOIHits/Test\ File/Participant_Part1_Condition_Hand_Trial_Howe.avi --subs --interactive --skip_intervals=00:00:10-00:00:15;00:00:20-00:00:25 --fixation=/home/chili/QTProjects/EyeTrackingExtractAOIHits/Test\ File/Participant_Part1_Condition_Hand_Trial_Howe.txt


