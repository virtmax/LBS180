QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
DEFINES += #QT_DISABLE_DEPRECATED_BEFORE=0x060000 \    # disables all the APIs deprecated before Qt 6.0.0
            NOMINMAX

SOURCES += \
    external/NucMath/src/datatable.cpp \
    external/NucMath/src/functions.cpp \
    external/NucMath/src/hist.cpp \
    external/NucMath/src/hist2.cpp \
    external/NucMath/src/hist3.cpp \
    external/NucMath/src/integration.cpp \
    external/NucMath/src/interpolation.cpp \
    external/NucMath/src/interpolationdata.cpp \
    external/NucMath/src/interpolationsteffen.cpp \
    external/NucMath/src/minimizer.cpp \
    external/NucMath/src/minimizerdownhillsimplex.cpp \
    external/NucMath/src/optpoint.cpp \
    external/NucMath/src/regression.cpp \
    external/NucMath/src/signalprocessing.cpp \
    external/NucMath/src/signalreconstruction.cpp \
    external/NucMath/src/stringoperations.cpp \
    external/NucMath/src/tablerow.cpp \
    external/NucMath/src/timeseries.cpp \
    external/NucMath/src/utils.cpp \
    external/qcustomplotzoom/qcustomplotzoom.cpp \
    external/qcustomplotzoom/qcustomplot/qcustomplot.cpp \
    main.cpp \
    mainwindow.cpp \
    source/rotposframe.cpp \
    external/brick_source/brick_silent_stepper.c \
    external/brick_source/bricklet_hall_effect_v2.c \
    external/brick_source/ip_connection.c

HEADERS += \
    external/NucMath/src/constants.h \
    external/NucMath/src/datatable.h \
    external/NucMath/src/functions.h \
    external/NucMath/src/hist.h \
    external/NucMath/src/hist2.h \
    external/NucMath/src/hist3.h \
    external/NucMath/src/integration.h \
    external/NucMath/src/interpolation.h \
    external/NucMath/src/interpolationdata.h \
    external/NucMath/src/interpolationsteffen.h \
    external/NucMath/src/matrix.h \
    external/NucMath/src/minimizer.h \
    external/NucMath/src/minimizerdef.h \
    external/NucMath/src/minimizerdownhillsimplex.h \
    external/NucMath/src/minimizerrandom.h \
    external/NucMath/src/optpoint.h \
    external/NucMath/src/regression.h \
    external/NucMath/src/scival.h \
    external/NucMath/src/signalprocessing.h \
    external/NucMath/src/signalreconstruction.h \
    external/NucMath/src/stringoperations.h \
    external/NucMath/src/tablerow.h \
    external/NucMath/src/timeseries.h \
    external/NucMath/src/utils.h \
    external/NucMath/src/vector.h \
    external/qcustomplotzoom/qcustomplotzoom.h \
    external/qcustomplotzoom/qcustomplot/qcustomplot.h \
    mainwindow.h \
    source/rotposframe.h \
    external/brick_source/brick_silent_stepper.h \
    external/brick_source/bricklet_hall_effect_v2.h \
    external/brick_source/ip_connection.h

FORMS += \
    mainwindow.ui

INCLUDEPATH +=  "./src/" \
                "./external/qcustomplotzoom/" \
                "./external/" \
                "./external/NucMath/src/" \
                "./external/NucMath/" \
		"./external/brick_source/" \

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


#TEMPLATE = app
#CONFIG += console
#TARGET = example_configuration
win32:LIBS += -lws2_32 -ladvapi32
unix:QMAKE_CXXFLAGS += -pthread
