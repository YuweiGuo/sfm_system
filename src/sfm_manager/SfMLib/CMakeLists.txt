cmake_minimum_required(VERSION 2.6)

file(GLOB SFM_LIB_SOURCES
        Virtual*.*
        CVFeatureMatcher.*
        EPGCameraPoseComputer.*
        PNPCameraPoseComputer.*
        LSTriangulator.*
        CVBundleAdjustor.*
        SfMTool.*
        BasicSetting.*
        SfMLibCameraPoseComputer.*
        SfMLibObject.*
        SfMLibFeatureMatcher.*
        SfMLibTriangulator.*
)

add_library(SfMLibrary ${SFM_LIB_SOURCES} )


