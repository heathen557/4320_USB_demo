#-------------------------------------------------
#
# Project created by QtCreator 2020-02-26T09:48:05
#
#-------------------------------------------------

QT       += core gui datavisualization multimedia multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = VI4320_USB_Line_1.1

TEMPLATE = app


OTHER_FILES += myapp.rc
RC_FILE += myapp.rc

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

LIBS += "libusb.lib"

SOURCES += main.cpp\
        mainwindow.cpp \
    mylabel.cpp \
    control_textedit.cpp \
    glwidget.cpp \
    logo.cpp \
    dealusb_msg.cpp \
    globaldata.cpp \
    savepcdthread.cpp \
    statisticsdialog.cpp \
    surfacegraph.cpp \
    filesave_dialog.cpp \
    calmeanstdthread.cpp \
    receusb_msg.cpp \
    autocalibration_dialog.cpp \
    aboutdialog.cpp \
    rawdataui_dialog.cpp \
    camerasetting_dialog.cpp \
    hist_ma_dialog.cpp \
    qcustomplot.cpp \
    filterwindow_dialog.cpp



HEADERS  += mainwindow.h \
    mylabel.h \
    control_textedit.h \
    globaldata.h \
    glwidget.h \
    logo.h \
    dealusb_msg.h \
    savepcdthread.h \
    statisticsdialog.h \
    surfacegraph.h \
    filesave_dialog.h \
    calmeanstdthread.h \
    receusb_msg.h \
    autocalibration_dialog.h \
    aboutdialog.h \
    rawdataui_dialog.h \
    camerasetting_dialog.h \
    hist_ma_dialog.h \
    qcustomplot.h \
    filterwindow_dialog.h

FORMS    += mainwindow.ui\
    statisticsdialog.ui \
    filesave_dialog.ui \
    autocalibration_dialog.ui \
    aboutdialog.ui \
    rawdataui_dialog.ui \
    camerasetting_dialog.ui \
    hist_ma_dialog.ui \
    filterwindow_dialog.ui


RESOURCES += \
    imgsource.qrc

#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/include/pcl-1.8\
INCLUDEPATH += D:/pcl/PCL1.8.1/include/pcl-1.8\



INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/include/pcl-1.8/pcl\
INCLUDEPATH += D:/pcl/PCL1.8.1/include/pcl-1.8/pcl\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/Boost/include/boost-1_59\
INCLUDEPATH += D:/pcl/PCL1.8.1/3rdParty/Boost/include/boost-1_64\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/Eigen/eigen3\
INCLUDEPATH += D:/pcl/PCL1.8.1/3rdParty/Eigen/eigen3\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/FLANN/include\
INCLUDEPATH += D:/pcl/PCL1.8.1/3rdParty/FLANN/include\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/FLANN/include/flann\
INCLUDEPATH += D:/pcl/PCL1.8.1/3rdParty/FLANN/include/flann\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/OpenNI2/Include\
INCLUDEPATH += D:/pcl/PCL1.8.1/3rdParty/OpenNI2/Include\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/Qhull/include\
INCLUDEPATH += D:/programInstall/pcl/PCL1.8.1/3rdParty/Qhull/include\


#INCLUDEPATH += C:/Tool/PCL/PCL1.8.0/3rdParty/VTK/include/vtk-7.0\
INCLUDEPATH += D:/pcl/PCL1.8.1/3rdParty/VTK/include/vtk-8.0\


CONFIG(debug,debug|release){
#LIBS += -LC:/Tool/PCL/PCL1.8.0/lib\
LIBS += -LD:/pcl/PCL1.8.1/lib\
        -lpcl_common_debug\
        -lpcl_features_debug\
        -lpcl_filters_debug\
        -lpcl_io_debug\
        -lpcl_io_ply_debug\
        -lpcl_kdtree_debug\
        -lpcl_keypoints_debug\
        -lpcl_ml_debug\
        -lpcl_octree_debug\
        -lpcl_outofcore_debug\
        -lpcl_people_debug\
        -lpcl_recognition_debug\
        -lpcl_registration_debug\
        -lpcl_sample_consensus_debug\
        -lpcl_search_debug\
        -lpcl_segmentation_debug\
#        -lpcl_simulation_debug\
        -lpcl_stereo_debug\
        -lpcl_surface_debug\
        -lpcl_tracking_debug\
        -lpcl_visualization_debug\

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/Boost/lib\
        -llibboost_atomic-vc140-mt-gd-1_64\
        -llibboost_bzip2-vc140-mt-gd-1_64\
        -llibboost_chrono-vc140-mt-gd-1_64\
        -llibboost_container-vc140-mt-gd-1_64\
        -llibboost_context-vc140-mt-gd-1_64\
        -llibboost_coroutine-vc140-mt-gd-1_64\
        -llibboost_date_time-vc140-mt-gd-1_64\
        -llibboost_exception-vc140-mt-gd-1_64\
        -llibboost_fiber-vc140-mt-gd-1_64\
        -llibboost_filesystem-vc140-mt-gd-1_64\
        -llibboost_graph_parallel-vc140-mt-gd-1_64\
        -llibboost_graph-vc140-mt-gd-1_64\
        -llibboost_iostreams-vc140-mt-gd-1_64\
        -llibboost_locale-vc140-mt-gd-1_64\
        -llibboost_log-vc140-mt-gd-1_64\
        -llibboost_log_setup-vc140-mt-gd-1_64\
        -llibboost_math_c99-vc140-mt-gd-1_64\
        -llibboost_math_c99f-vc140-mt-gd-1_64\
        -llibboost_math_c99l-vc140-mt-gd-1_64\
        -llibboost_math_tr1-vc140-mt-gd-1_64\
        -llibboost_math_tr1f-vc140-mt-gd-1_64\
        -llibboost_math_tr1l-vc140-mt-gd-1_64\
        -llibboost_mpi-vc140-mt-gd-1_64\
        -llibboost_numpy3-vc140-mt-gd-1_64\
        -llibboost_numpy-vc140-mt-gd-1_64\
        -llibboost_prg_exec_monitor-vc140-mt-gd-1_64\
        -llibboost_program_options-vc140-mt-gd-1_64\
        -llibboost_python3-vc140-mt-gd-1_64\
        -llibboost_python-vc140-mt-gd-1_64\
        -llibboost_random-vc140-mt-gd-1_64\
        -llibboost_regex-vc140-mt-gd-1_64\
        -llibboost_serialization-vc140-mt-gd-1_64\
        -llibboost_signals-vc140-mt-gd-1_64\
        -llibboost_system-vc140-mt-gd-1_64\
        -llibboost_test_exec_monitor-vc140-mt-gd-1_64\
        -llibboost_thread-vc140-mt-gd-1_64\
        -llibboost_timer-vc140-mt-gd-1_64\
        -llibboost_type_erasure-vc140-mt-gd-1_64\
        -llibboost_unit_test_framework-vc140-mt-gd-1_64\
        -llibboost_wave-vc140-mt-gd-1_64\
        -llibboost_wserialization-vc140-mt-gd-1_64\
        -llibboost_zlib-vc140-mt-gd-1_64

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/FLANN/lib\
        -lflann-gd\
        -lflann_cpp_s-gd\
        -lflann_s-gd\
        -lflann_cpp-gd


LIBS += -LD:/pcl/PCL1.8.1/3rdParty/OpenNI2/Lib\
        -lOpenNI2

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/Qhull/lib\
        -lqhull\
        -lqhull_d\
        -lqhull_p\
        -lqhull_p_d\
        -lqhull_r\
        -lqhull_r_d\
        -lqhullcpp\
        -lqhullcpp_d\
        -lqhullstatic\
        -lqhullstatic_d\
        -lqhullstatic_r\
        -lqhullstatic_r_d\

LIBS += -LD:/pcl/PCL1.8.1\3rdParty\VTK\lib\
        -lvtkalglib-8.0-gd\
        -lvtkChartsCore-8.0-gd\
        -lvtkCommonColor-8.0-gd\
        -lvtkCommonComputationalGeometry-8.0-gd\
        -lvtkCommonCore-8.0-gd\
        -lvtkCommonDataModel-8.0-gd\
        -lvtkCommonExecutionModel-8.0-gd\
        -lvtkCommonMath-8.0-gd\
        -lvtkCommonMisc-8.0-gd\
        -lvtkCommonSystem-8.0-gd\
        -lvtkCommonTransforms-8.0-gd\
        -lvtkDICOMParser-8.0-gd\
        -lvtkDomainsChemistry-8.0-gd\
#        -lvtkDomainsChemistryOpenGL2-8.0-gd\
        -lvtkexoIIc-8.0-gd\
        -lvtkexpat-8.0-gd\
        -lvtkFiltersAMR-8.0-gd\
        -lvtkFiltersCore-8.0-gd\
        -lvtkFiltersExtraction-8.0-gd\
        -lvtkFiltersFlowPaths-8.0-gd\
        -lvtkFiltersGeneral-8.0-gd\
        -lvtkFiltersGeneric-8.0-gd\
        -lvtkFiltersGeometry-8.0-gd\
        -lvtkFiltersHybrid-8.0-gd\
        -lvtkFiltersHyperTree-8.0-gd\
        -lvtkFiltersImaging-8.0-gd\
        -lvtkFiltersModeling-8.0-gd\
        -lvtkFiltersParallel-8.0-gd\
        -lvtkFiltersParallelImaging-8.0-gd\
        -lvtkFiltersPoints-8.0-gd\
        -lvtkFiltersProgrammable-8.0-gd\
        -lvtkFiltersSelection-8.0-gd\
        -lvtkFiltersSMP-8.0-gd\
        -lvtkFiltersSources-8.0-gd\
        -lvtkFiltersStatistics-8.0-gd\
        -lvtkFiltersTexture-8.0-gd\
        -lvtkFiltersTopology-8.0-gd\
        -lvtkFiltersVerdict-8.0-gd\
        -lvtkfreetype-8.0-gd\
        -lvtkGeovisCore-8.0-gd\
#        -lvtkglew-8.0-gd\
#        -lvtkGUISupportQt-8.0-gd\
#        -lvtkGUISupportQtSQL-8.0-gd\
#        -lvtkGeovisCore-8.0-gd\
        -lvtkgl2ps-8.0-gd\
        -lvtkhdf5-8.0-gd\
        -lvtkhdf5_hl-8.0-gd\
        -lvtkImagingColor-8.0-gd\
        -lvtkImagingCore-8.0-gd\
        -lvtkImagingFourier-8.0-gd\
        -lvtkImagingGeneral-8.0-gd\
        -lvtkImagingHybrid-8.0-gd\
        -lvtkImagingMath-8.0-gd\
        -lvtkImagingMorphological-8.0-gd\
        -lvtkImagingSources-8.0-gd\
        -lvtkImagingStatistics-8.0-gd\
        -lvtkImagingStencil-8.0-gd\
        -lvtkInfovisCore-8.0-gd\
        -lvtkInfovisLayout-8.0-gd\
        -lvtkInteractionImage-8.0-gd\
        -lvtkInteractionStyle-8.0-gd\
        -lvtkInteractionWidgets-8.0-gd\
        -lvtkIOAMR-8.0-gd\
        -lvtkIOCore-8.0-gd\
        -lvtkIOEnSight-8.0-gd\
        -lvtkIOExodus-8.0-gd\
        -lvtkIOExport-8.0-gd\
        -lvtkIOGeometry-8.0-gd\
        -lvtkIOImage-8.0-gd\
        -lvtkIOImport-8.0-gd\
        -lvtkIOInfovis-8.0-gd\
        -lvtkIOLegacy-8.0-gd\
        -lvtkIOLSDyna-8.0-gd\
        -lvtkIOMINC-8.0-gd\
        -lvtkIOMovie-8.0-gd\
        -lvtkIONetCDF-8.0-gd\
        -lvtkIOParallel-8.0-gd\
        -lvtkIOParallelXML-8.0-gd\
        -lvtkIOPLY-8.0-gd\
        -lvtkIOSQL-8.0-gd\
        -lvtkIOTecplotTable-8.0-gd\
        -lvtkIOVideo-8.0-gd\
        -lvtkIOXML-8.0-gd\
        -lvtkIOXMLParser-8.0-gd\
        -lvtkjpeg-8.0-gd\
        -lvtkjsoncpp-8.0-gd\
        -lvtklibharu-8.0-gd\
        -lvtklibxml2-8.0-gd\
        -lvtklz4-8.0-gd\
        -lvtkmetaio-8.0-gd\
        -lvtkNetCDF-8.0-gd\
#        -lvtkNetCDF_cxx-8.0-gd\
        -lvtknetcdf_c++-gd\
        -lvtkoggtheora-8.0-gd\
        -lvtkParallelCore-8.0-gd\
        -lvtkpng-8.0-gd\
        -lvtkproj4-8.0-gd\
        -lvtkRenderingAnnotation-8.0-gd\
        -lvtkRenderingContext2D-8.0-gd\
#        -lvtkRenderingContextOpenGL2-8.0-gd\
        -lvtkRenderingContextOpenGL-8.0-gd\
        -lvtkRenderingCore-8.0-gd\
        -lvtkRenderingFreeType-8.0-gd\
        -lvtkRenderingGL2PS-8.0-gd\
        -lvtkRenderingImage-8.0-gd\
        -lvtkRenderingLabel-8.0-gd\
        -lvtkRenderingLIC-8.0-gd\
        -lvtkRenderingLOD-8.0-gd\
#        -lvtkRenderingOpenGL2-8.0-gd\
        -lvtkRenderingOpenGL-8.0-gd\
#        -lvtkRenderingQt-8.0-gd\
        -lvtkRenderingVolume-8.0-gd\
#        -lvtkRenderingVolumeOpenGL2-8.0-gd\
        -lvtkRenderingVolumeOpenGL-8.0-gd\
        -lvtksqlite-8.0-gd\
        -lvtksys-8.0-gd\
        -lvtktiff-8.0-gd\
        -lvtkverdict-8.0-gd\
        -lvtkViewsContext2D-8.0-gd\
        -lvtkViewsCore-8.0-gd\
        -lvtkViewsInfovis-8.0-gd\
#        -lvtkViewsQt-8.0-gd\
        -lvtkzlib-8.0-gd

} else {
LIBS += -LD:/pcl/PCL1.8.1/lib\
        -lpcl_common_release\
        -lpcl_features_release\
        -lpcl_filters_release\
        -lpcl_io_release\
        -lpcl_io_ply_release\
        -lpcl_kdtree_release\
        -lpcl_keypoints_release\
        -lpcl_ml_release\
        -lpcl_octree_release\
        -lpcl_outofcore_release\
        -lpcl_people_release\
        -lpcl_recognition_release\
        -lpcl_registration_release\
        -lpcl_sample_consensus_release\
        -lpcl_search_release\
        -lpcl_segmentation_release\
        -lpcl_stereo_release\
        -lpcl_surface_release\
        -lpcl_tracking_release\
        -lpcl_visualization_release\

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/Boost/lib\
        -llibboost_atomic-vc140-mt-1_64\
        -llibboost_bzip2-vc140-mt-1_64\
        -llibboost_chrono-vc140-mt-1_64\
        -llibboost_container-vc140-mt-1_64\
        -llibboost_context-vc140-mt-1_64\
        -llibboost_coroutine-vc140-mt-1_64\
        -llibboost_date_time-vc140-mt-1_64\
        -llibboost_exception-vc140-mt-1_64\
        -llibboost_fiber-vc140-mt-1_64\
        -llibboost_filesystem-vc140-mt-1_64\
        -llibboost_graph_parallel-vc140-mt-1_64\
        -llibboost_graph-vc140-mt-1_64\
        -llibboost_iostreams-vc140-mt-1_64\
        -llibboost_locale-vc140-mt-1_64\
        -llibboost_log-vc140-mt-1_64\
        -llibboost_log_setup-vc140-mt-1_64\
        -llibboost_math_c99-vc140-mt-1_64\
        -llibboost_math_c99f-vc140-mt-1_64\
        -llibboost_math_c99l-vc140-mt-1_64\
        -llibboost_math_tr1-vc140-mt-1_64\
        -llibboost_math_tr1f-vc140-mt-1_64\
        -llibboost_math_tr1l-vc140-mt-1_64\
        -llibboost_mpi-vc140-mt-1_64\
        -llibboost_numpy3-vc140-mt-1_64\
        -llibboost_numpy-vc140-mt-1_64\
        -llibboost_prg_exec_monitor-vc140-mt-1_64\
        -llibboost_program_options-vc140-mt-1_64\
        -llibboost_python3-vc140-mt-1_64\
        -llibboost_python-vc140-mt-1_64\
        -llibboost_random-vc140-mt-1_64\
        -llibboost_regex-vc140-mt-1_64\
        -llibboost_serialization-vc140-mt-1_64\
        -llibboost_signals-vc140-mt-1_64\
        -llibboost_system-vc140-mt-1_64\
        -llibboost_test_exec_monitor-vc140-mt-1_64\
        -llibboost_thread-vc140-mt-1_64\
        -llibboost_timer-vc140-mt-1_64\
        -llibboost_type_erasure-vc140-mt-1_64\
        -llibboost_unit_test_framework-vc140-mt-1_64\
        -llibboost_wave-vc140-mt-1_64\
        -llibboost_wserialization-vc140-mt-1_64\
        -llibboost_zlib-vc140-mt-1_64

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/FLANN/lib\
        -lflann\
        -lflann_cpp_s\
        -lflann_s

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/OpenNI2/Lib\
        -lOpenNI2

LIBS += -LD:/pcl/PCL1.8.1/3rdParty/Qhull/lib\
        -lqhull\
        -lqhullcpp\
        -lqhullstatic\
        -lqhullstatic_r\
        -lqhull_p\
        -lqhull_r

LIBS += -LD:/pcl/PCL1.8.1\3rdParty\VTK\lib\
        -lvtkalglib-8.0\
        -lvtkChartsCore-8.0\
        -lvtkCommonColor-8.0\
        -lvtkCommonComputationalGeometry-8.0\
        -lvtkCommonCore-8.0\
        -lvtkCommonDataModel-8.0\
        -lvtkCommonExecutionModel-8.0\
        -lvtkCommonMath-8.0\
        -lvtkCommonMisc-8.0\
        -lvtkCommonSystem-8.0\
        -lvtkCommonTransforms-8.0\
        -lvtkDICOMParser-8.0\
        -lvtkDomainsChemistry-8.0\
#        -lvtkDomainsChemistryOpenGL2-8.0\
        -lvtkexoIIc-8.0\
        -lvtkexpat-8.0\
        -lvtkFiltersAMR-8.0\
        -lvtkFiltersCore-8.0\
        -lvtkFiltersExtraction-8.0\
        -lvtkFiltersFlowPaths-8.0\
        -lvtkFiltersGeneral-8.0\
        -lvtkFiltersGeneric-8.0\
        -lvtkFiltersGeometry-8.0\
        -lvtkFiltersHybrid-8.0\
        -lvtkFiltersHyperTree-8.0\
        -lvtkFiltersImaging-8.0\
        -lvtkFiltersModeling-8.0\
        -lvtkFiltersParallel-8.0\
        -lvtkFiltersParallelImaging-8.0\
        -lvtkFiltersPoints-8.0\
        -lvtkFiltersProgrammable-8.0\
        -lvtkFiltersSelection-8.0\
        -lvtkFiltersSMP-8.0\
        -lvtkFiltersSources-8.0\
        -lvtkFiltersStatistics-8.0\
        -lvtkFiltersTexture-8.0\
        -lvtkFiltersTopology-8.0\
        -lvtkFiltersVerdict-8.0\
        -lvtkfreetype-8.0\
        -lvtkGeovisCore-8.0\
#        -lvtkglew-8.0\
#        -lvtkGUISupportQt-8.0\
#        -lvtkGUISupportQtSQL-8.0\
#        -lvtkGeovisCore-8.0\
        -lvtkgl2ps-8.0\
        -lvtkhdf5-8.0\
        -lvtkhdf5_hl-8.0\
        -lvtkImagingColor-8.0\
        -lvtkImagingCore-8.0\
        -lvtkImagingFourier-8.0\
        -lvtkImagingGeneral-8.0\
        -lvtkImagingHybrid-8.0\
        -lvtkImagingMath-8.0\
        -lvtkImagingMorphological-8.0\
        -lvtkImagingSources-8.0\
        -lvtkImagingStatistics-8.0\
        -lvtkImagingStencil-8.0\
        -lvtkInfovisCore-8.0\
        -lvtkInfovisLayout-8.0\
        -lvtkInteractionImage-8.0\
        -lvtkInteractionStyle-8.0\
        -lvtkInteractionWidgets-8.0\
        -lvtkIOAMR-8.0\
        -lvtkIOCore-8.0\
        -lvtkIOEnSight-8.0\
        -lvtkIOExodus-8.0\
        -lvtkIOExport-8.0\
        -lvtkIOGeometry-8.0\
        -lvtkIOImage-8.0\
        -lvtkIOImport-8.0\
        -lvtkIOInfovis-8.0\
        -lvtkIOLegacy-8.0\
        -lvtkIOLSDyna-8.0\
        -lvtkIOMINC-8.0\
        -lvtkIOMovie-8.0\
        -lvtkIONetCDF-8.0\
        -lvtkIOParallel-8.0\
        -lvtkIOParallelXML-8.0\
        -lvtkIOPLY-8.0\
        -lvtkIOSQL-8.0\
        -lvtkIOTecplotTable-8.0\
        -lvtkIOVideo-8.0\
        -lvtkIOXML-8.0\
        -lvtkIOXMLParser-8.0\
        -lvtkjpeg-8.0\
        -lvtkjsoncpp-8.0\
        -lvtklibharu-8.0\
        -lvtklibxml2-8.0\
        -lvtklz4-8.0\
        -lvtkmetaio-8.0\
        -lvtkNetCDF-8.0\
#        -lvtkNetCDF_cxx-8.0\
        -lvtknetcdf_c++\
        -lvtkoggtheora-8.0\
        -lvtkParallelCore-8.0\
        -lvtkpng-8.0\
        -lvtkproj4-8.0\
        -lvtkRenderingAnnotation-8.0\
        -lvtkRenderingContext2D-8.0\
#        -lvtkRenderingContextOpenGL2-8.0\
        -lvtkRenderingContextOpenGL-8.0\
        -lvtkRenderingCore-8.0\
        -lvtkRenderingFreeType-8.0\
        -lvtkRenderingGL2PS-8.0\
        -lvtkRenderingImage-8.0\
        -lvtkRenderingLabel-8.0\
        -lvtkRenderingLIC-8.0\
        -lvtkRenderingLOD-8.0\
#        -lvtkRenderingOpenGL2-8.0\
        -lvtkRenderingOpenGL-8.0\
#        -lvtkRenderingQt-8.0\
        -lvtkRenderingVolume-8.0\
#        -lvtkRenderingVolumeOpenGL2-8.0\
        -lvtkRenderingVolumeOpenGL-8.0\
        -lvtksqlite-8.0\
        -lvtksys-8.0\
        -lvtktiff-8.0\
        -lvtkverdict-8.0\
        -lvtkViewsContext2D-8.0\
        -lvtkViewsCore-8.0\
        -lvtkViewsInfovis-8.0\
#        -lvtkViewsQt-8.0\
        -lvtkzlib-8.0
}



