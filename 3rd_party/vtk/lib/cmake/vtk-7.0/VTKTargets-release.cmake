#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vtksys" for configuration "Release"
set_property(TARGET vtksys APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtksys PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtksys-7.0.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "ws2_32;Psapi"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtksys-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtksys )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtksys "${_IMPORT_PREFIX}/lib/vtksys-7.0.lib" "${_IMPORT_PREFIX}/bin/vtksys-7.0.dll" )

# Import target "vtkCommonCore" for configuration "Release"
set_property(TARGET vtkCommonCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonCore-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonCore "${_IMPORT_PREFIX}/lib/vtkCommonCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonCore-7.0.dll" )

# Import target "vtkCommonMath" for configuration "Release"
set_property(TARGET vtkCommonMath APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonMath PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonMath-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonMath-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonMath )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonMath "${_IMPORT_PREFIX}/lib/vtkCommonMath-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonMath-7.0.dll" )

# Import target "vtkCommonMisc" for configuration "Release"
set_property(TARGET vtkCommonMisc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonMisc PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonMisc-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonMisc-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonMisc )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonMisc "${_IMPORT_PREFIX}/lib/vtkCommonMisc-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonMisc-7.0.dll" )

# Import target "vtkCommonSystem" for configuration "Release"
set_property(TARGET vtkCommonSystem APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonSystem PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonSystem-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonSystem-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonSystem )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonSystem "${_IMPORT_PREFIX}/lib/vtkCommonSystem-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonSystem-7.0.dll" )

# Import target "vtkCommonTransforms" for configuration "Release"
set_property(TARGET vtkCommonTransforms APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonTransforms PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonTransforms-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonTransforms-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonTransforms )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonTransforms "${_IMPORT_PREFIX}/lib/vtkCommonTransforms-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonTransforms-7.0.dll" )

# Import target "vtkCommonDataModel" for configuration "Release"
set_property(TARGET vtkCommonDataModel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonDataModel PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonDataModel-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonDataModel-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonDataModel )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonDataModel "${_IMPORT_PREFIX}/lib/vtkCommonDataModel-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonDataModel-7.0.dll" )

# Import target "vtkCommonColor" for configuration "Release"
set_property(TARGET vtkCommonColor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonColor PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonColor-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonColor-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonColor )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonColor "${_IMPORT_PREFIX}/lib/vtkCommonColor-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonColor-7.0.dll" )

# Import target "vtkCommonExecutionModel" for configuration "Release"
set_property(TARGET vtkCommonExecutionModel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonExecutionModel PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonExecutionModel-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonExecutionModel-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonExecutionModel )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonExecutionModel "${_IMPORT_PREFIX}/lib/vtkCommonExecutionModel-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonExecutionModel-7.0.dll" )

# Import target "vtkFiltersCore" for configuration "Release"
set_property(TARGET vtkFiltersCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersCore-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersCore "${_IMPORT_PREFIX}/lib/vtkFiltersCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersCore-7.0.dll" )

# Import target "vtkCommonComputationalGeometry" for configuration "Release"
set_property(TARGET vtkCommonComputationalGeometry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkCommonComputationalGeometry PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkCommonComputationalGeometry-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkCommonComputationalGeometry-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkCommonComputationalGeometry )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkCommonComputationalGeometry "${_IMPORT_PREFIX}/lib/vtkCommonComputationalGeometry-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkCommonComputationalGeometry-7.0.dll" )

# Import target "vtkFiltersGeneral" for configuration "Release"
set_property(TARGET vtkFiltersGeneral APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersGeneral PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersGeneral-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersGeneral-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersGeneral )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersGeneral "${_IMPORT_PREFIX}/lib/vtkFiltersGeneral-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersGeneral-7.0.dll" )

# Import target "vtkImagingCore" for configuration "Release"
set_property(TARGET vtkImagingCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingCore-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingCore "${_IMPORT_PREFIX}/lib/vtkImagingCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingCore-7.0.dll" )

# Import target "vtkImagingFourier" for configuration "Release"
set_property(TARGET vtkImagingFourier APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingFourier PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingFourier-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingFourier-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingFourier )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingFourier "${_IMPORT_PREFIX}/lib/vtkImagingFourier-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingFourier-7.0.dll" )

# Import target "vtkalglib" for configuration "Release"
set_property(TARGET vtkalglib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkalglib PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkalglib-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkalglib-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkalglib )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkalglib "${_IMPORT_PREFIX}/lib/vtkalglib-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkalglib-7.0.dll" )

# Import target "vtkFiltersStatistics" for configuration "Release"
set_property(TARGET vtkFiltersStatistics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersStatistics PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersStatistics-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersStatistics-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersStatistics )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersStatistics "${_IMPORT_PREFIX}/lib/vtkFiltersStatistics-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersStatistics-7.0.dll" )

# Import target "vtkFiltersExtraction" for configuration "Release"
set_property(TARGET vtkFiltersExtraction APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersExtraction PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersExtraction-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersExtraction-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersExtraction )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersExtraction "${_IMPORT_PREFIX}/lib/vtkFiltersExtraction-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersExtraction-7.0.dll" )

# Import target "vtkInfovisCore" for configuration "Release"
set_property(TARGET vtkInfovisCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkInfovisCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkInfovisCore-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkInfovisCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkInfovisCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkInfovisCore "${_IMPORT_PREFIX}/lib/vtkInfovisCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkInfovisCore-7.0.dll" )

# Import target "vtkFiltersGeometry" for configuration "Release"
set_property(TARGET vtkFiltersGeometry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersGeometry PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersGeometry-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersGeometry-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersGeometry )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersGeometry "${_IMPORT_PREFIX}/lib/vtkFiltersGeometry-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersGeometry-7.0.dll" )

# Import target "vtkFiltersSources" for configuration "Release"
set_property(TARGET vtkFiltersSources APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersSources PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersSources-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersSources-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersSources )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersSources "${_IMPORT_PREFIX}/lib/vtkFiltersSources-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersSources-7.0.dll" )

# Import target "vtkRenderingCore" for configuration "Release"
set_property(TARGET vtkRenderingCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingCore-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersSources;vtkFiltersGeometry;vtkFiltersExtraction;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingCore "${_IMPORT_PREFIX}/lib/vtkRenderingCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingCore-7.0.dll" )

# Import target "vtkzlib" for configuration "Release"
set_property(TARGET vtkzlib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkzlib PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkzlib-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkzlib-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkzlib )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkzlib "${_IMPORT_PREFIX}/lib/vtkzlib-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkzlib-7.0.dll" )

# Import target "vtkfreetype" for configuration "Release"
set_property(TARGET vtkfreetype APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkfreetype PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkfreetype-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkfreetype-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkfreetype )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkfreetype "${_IMPORT_PREFIX}/lib/vtkfreetype-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkfreetype-7.0.dll" )

# Import target "vtkRenderingFreeType" for configuration "Release"
set_property(TARGET vtkRenderingFreeType APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingFreeType PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingFreeType-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingFreeType-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingFreeType )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingFreeType "${_IMPORT_PREFIX}/lib/vtkRenderingFreeType-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingFreeType-7.0.dll" )

# Import target "vtkRenderingContext2D" for configuration "Release"
set_property(TARGET vtkRenderingContext2D APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingContext2D PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingContext2D-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkCommonDataModel;vtkCommonMath;vtkCommonTransforms;vtkRenderingFreeType"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingContext2D-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingContext2D )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingContext2D "${_IMPORT_PREFIX}/lib/vtkRenderingContext2D-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingContext2D-7.0.dll" )

# Import target "vtkChartsCore" for configuration "Release"
set_property(TARGET vtkChartsCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkChartsCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkChartsCore-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkCommonColor;vtkInfovisCore"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkChartsCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkChartsCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkChartsCore "${_IMPORT_PREFIX}/lib/vtkChartsCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkChartsCore-7.0.dll" )

# Import target "vtkDICOMParser" for configuration "Release"
set_property(TARGET vtkDICOMParser APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkDICOMParser PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkDICOMParser-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkDICOMParser-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkDICOMParser )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkDICOMParser "${_IMPORT_PREFIX}/lib/vtkDICOMParser-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkDICOMParser-7.0.dll" )

# Import target "vtkIOCore" for configuration "Release"
set_property(TARGET vtkIOCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOCore-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkzlib;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOCore "${_IMPORT_PREFIX}/lib/vtkIOCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOCore-7.0.dll" )

# Import target "vtkIOGeometry" for configuration "Release"
set_property(TARGET vtkIOGeometry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOGeometry PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOGeometry-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkzlib;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOGeometry-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOGeometry )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOGeometry "${_IMPORT_PREFIX}/lib/vtkIOGeometry-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOGeometry-7.0.dll" )

# Import target "vtkexpat" for configuration "Release"
set_property(TARGET vtkexpat APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkexpat PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkexpat-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkexpat-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkexpat )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkexpat "${_IMPORT_PREFIX}/lib/vtkexpat-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkexpat-7.0.dll" )

# Import target "vtkIOXMLParser" for configuration "Release"
set_property(TARGET vtkIOXMLParser APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOXMLParser PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOXMLParser-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkexpat"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOXMLParser-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOXMLParser )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOXMLParser "${_IMPORT_PREFIX}/lib/vtkIOXMLParser-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOXMLParser-7.0.dll" )

# Import target "vtkIOXML" for configuration "Release"
set_property(TARGET vtkIOXML APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOXML PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOXML-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOXML-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOXML )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOXML "${_IMPORT_PREFIX}/lib/vtkIOXML-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOXML-7.0.dll" )

# Import target "vtkDomainsChemistry" for configuration "Release"
set_property(TARGET vtkDomainsChemistry APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkDomainsChemistry PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkDomainsChemistry-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkIOXML;vtkFiltersSources"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkDomainsChemistry-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkDomainsChemistry )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkDomainsChemistry "${_IMPORT_PREFIX}/lib/vtkDomainsChemistry-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkDomainsChemistry-7.0.dll" )

# Import target "vtkIOLegacy" for configuration "Release"
set_property(TARGET vtkIOLegacy APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOLegacy PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOLegacy-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOLegacy-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOLegacy )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOLegacy "${_IMPORT_PREFIX}/lib/vtkIOLegacy-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOLegacy-7.0.dll" )

# Import target "vtkHashSource" for configuration "Release"
set_property(TARGET vtkHashSource APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkHashSource PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkHashSource-7.0.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkHashSource )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkHashSource "${_IMPORT_PREFIX}/bin/vtkHashSource-7.0.exe" )

# Import target "vtkParallelCore" for configuration "Release"
set_property(TARGET vtkParallelCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkParallelCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkParallelCore-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkParallelCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkParallelCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkParallelCore "${_IMPORT_PREFIX}/lib/vtkParallelCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkParallelCore-7.0.dll" )

# Import target "vtkFiltersAMR" for configuration "Release"
set_property(TARGET vtkFiltersAMR APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersAMR PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersAMR-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersAMR-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersAMR )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersAMR "${_IMPORT_PREFIX}/lib/vtkFiltersAMR-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersAMR-7.0.dll" )

# Import target "vtkFiltersFlowPaths" for configuration "Release"
set_property(TARGET vtkFiltersFlowPaths APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersFlowPaths PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersFlowPaths-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersFlowPaths-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersFlowPaths )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersFlowPaths "${_IMPORT_PREFIX}/lib/vtkFiltersFlowPaths-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersFlowPaths-7.0.dll" )

# Import target "vtkFiltersGeneric" for configuration "Release"
set_property(TARGET vtkFiltersGeneric APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersGeneric PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersGeneric-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersGeneric-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersGeneric )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersGeneric "${_IMPORT_PREFIX}/lib/vtkFiltersGeneric-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersGeneric-7.0.dll" )

# Import target "vtkImagingSources" for configuration "Release"
set_property(TARGET vtkImagingSources APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingSources PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingSources-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingSources-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingSources )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingSources "${_IMPORT_PREFIX}/lib/vtkImagingSources-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingSources-7.0.dll" )

# Import target "vtkFiltersHybrid" for configuration "Release"
set_property(TARGET vtkFiltersHybrid APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersHybrid PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersHybrid-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersHybrid-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersHybrid )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersHybrid "${_IMPORT_PREFIX}/lib/vtkFiltersHybrid-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersHybrid-7.0.dll" )

# Import target "vtkFiltersHyperTree" for configuration "Release"
set_property(TARGET vtkFiltersHyperTree APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersHyperTree PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersHyperTree-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersHyperTree-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersHyperTree )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersHyperTree "${_IMPORT_PREFIX}/lib/vtkFiltersHyperTree-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersHyperTree-7.0.dll" )

# Import target "vtkImagingGeneral" for configuration "Release"
set_property(TARGET vtkImagingGeneral APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingGeneral PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingGeneral-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingGeneral-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingGeneral )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingGeneral "${_IMPORT_PREFIX}/lib/vtkImagingGeneral-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingGeneral-7.0.dll" )

# Import target "vtkFiltersImaging" for configuration "Release"
set_property(TARGET vtkFiltersImaging APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersImaging PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersImaging-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersImaging-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersImaging )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersImaging "${_IMPORT_PREFIX}/lib/vtkFiltersImaging-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersImaging-7.0.dll" )

# Import target "vtkFiltersModeling" for configuration "Release"
set_property(TARGET vtkFiltersModeling APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersModeling PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersModeling-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersModeling-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersModeling )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersModeling "${_IMPORT_PREFIX}/lib/vtkFiltersModeling-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersModeling-7.0.dll" )

# Import target "vtkFiltersParallel" for configuration "Release"
set_property(TARGET vtkFiltersParallel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersParallel PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersParallel-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersParallel-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersParallel )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersParallel "${_IMPORT_PREFIX}/lib/vtkFiltersParallel-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersParallel-7.0.dll" )

# Import target "vtkFiltersParallelImaging" for configuration "Release"
set_property(TARGET vtkFiltersParallelImaging APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersParallelImaging PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersParallelImaging-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersParallelImaging-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersParallelImaging )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersParallelImaging "${_IMPORT_PREFIX}/lib/vtkFiltersParallelImaging-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersParallelImaging-7.0.dll" )

# Import target "vtkFiltersProgrammable" for configuration "Release"
set_property(TARGET vtkFiltersProgrammable APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersProgrammable PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersProgrammable-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersProgrammable-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersProgrammable )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersProgrammable "${_IMPORT_PREFIX}/lib/vtkFiltersProgrammable-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersProgrammable-7.0.dll" )

# Import target "vtkFiltersSMP" for configuration "Release"
set_property(TARGET vtkFiltersSMP APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersSMP PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersSMP-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersSMP-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersSMP )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersSMP "${_IMPORT_PREFIX}/lib/vtkFiltersSMP-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersSMP-7.0.dll" )

# Import target "vtkFiltersSelection" for configuration "Release"
set_property(TARGET vtkFiltersSelection APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersSelection PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersSelection-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersSelection-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersSelection )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersSelection "${_IMPORT_PREFIX}/lib/vtkFiltersSelection-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersSelection-7.0.dll" )

# Import target "vtkFiltersTexture" for configuration "Release"
set_property(TARGET vtkFiltersTexture APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersTexture PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersTexture-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersTexture-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersTexture )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersTexture "${_IMPORT_PREFIX}/lib/vtkFiltersTexture-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersTexture-7.0.dll" )

# Import target "verdict" for configuration "Release"
set_property(TARGET verdict APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(verdict PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkverdict-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkverdict-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS verdict )
list(APPEND _IMPORT_CHECK_FILES_FOR_verdict "${_IMPORT_PREFIX}/lib/vtkverdict-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkverdict-7.0.dll" )

# Import target "vtkFiltersVerdict" for configuration "Release"
set_property(TARGET vtkFiltersVerdict APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkFiltersVerdict PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkFiltersVerdict-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkFiltersVerdict-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkFiltersVerdict )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkFiltersVerdict "${_IMPORT_PREFIX}/lib/vtkFiltersVerdict-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkFiltersVerdict-7.0.dll" )

# Import target "vtkInteractionStyle" for configuration "Release"
set_property(TARGET vtkInteractionStyle APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkInteractionStyle PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkInteractionStyle-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersSources;vtkFiltersExtraction"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkInteractionStyle-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkInteractionStyle )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkInteractionStyle "${_IMPORT_PREFIX}/lib/vtkInteractionStyle-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkInteractionStyle-7.0.dll" )

# Import target "vtkmetaio" for configuration "Release"
set_property(TARGET vtkmetaio APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkmetaio PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkmetaio-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkmetaio-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkmetaio )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkmetaio "${_IMPORT_PREFIX}/lib/vtkmetaio-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkmetaio-7.0.dll" )

# Import target "vtkjpeg" for configuration "Release"
set_property(TARGET vtkjpeg APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkjpeg PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkjpeg-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkjpeg-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkjpeg )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkjpeg "${_IMPORT_PREFIX}/lib/vtkjpeg-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkjpeg-7.0.dll" )

# Import target "vtkpng" for configuration "Release"
set_property(TARGET vtkpng APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkpng PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkpng-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkpng-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkpng )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkpng "${_IMPORT_PREFIX}/lib/vtkpng-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkpng-7.0.dll" )

# Import target "vtktiff" for configuration "Release"
set_property(TARGET vtktiff APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtktiff PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtktiff-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtktiff-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtktiff )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtktiff "${_IMPORT_PREFIX}/lib/vtktiff-7.0.lib" "${_IMPORT_PREFIX}/bin/vtktiff-7.0.dll" )

# Import target "vtkIOImage" for configuration "Release"
set_property(TARGET vtkIOImage APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOImage PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOImage-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkjpeg;vtkpng;vtktiff;vtkmetaio;vtkDICOMParser;vtkzlib;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOImage-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOImage )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOImage "${_IMPORT_PREFIX}/lib/vtkIOImage-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOImage-7.0.dll" )

# Import target "vtkImagingHybrid" for configuration "Release"
set_property(TARGET vtkImagingHybrid APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingHybrid PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingHybrid-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingHybrid-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingHybrid )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingHybrid "${_IMPORT_PREFIX}/lib/vtkImagingHybrid-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingHybrid-7.0.dll" )

# Import target "vtkParseOGLExt" for configuration "Release"
set_property(TARGET vtkParseOGLExt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkParseOGLExt PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkParseOGLExt-7.0.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkParseOGLExt )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkParseOGLExt "${_IMPORT_PREFIX}/bin/vtkParseOGLExt-7.0.exe" )

# Import target "vtkEncodeString" for configuration "Release"
set_property(TARGET vtkEncodeString APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkEncodeString PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkEncodeString-7.0.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkEncodeString )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkEncodeString "${_IMPORT_PREFIX}/bin/vtkEncodeString-7.0.exe" )

# Import target "vtkRenderingOpenGL" for configuration "Release"
set_property(TARGET vtkRenderingOpenGL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingOpenGL PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingOpenGL-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkImagingHybrid;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingOpenGL-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingOpenGL )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingOpenGL "${_IMPORT_PREFIX}/lib/vtkRenderingOpenGL-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingOpenGL-7.0.dll" )

# Import target "vtkGUISupportQt" for configuration "Release"
set_property(TARGET vtkGUISupportQt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGUISupportQt PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkGUISupportQt-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersExtraction"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkGUISupportQt-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkGUISupportQt )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkGUISupportQt "${_IMPORT_PREFIX}/lib/vtkGUISupportQt-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkGUISupportQt-7.0.dll" )

# Import target "vtkGUISupportQtOpenGL" for configuration "Release"
set_property(TARGET vtkGUISupportQtOpenGL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGUISupportQtOpenGL PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkGUISupportQtOpenGL-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "Qt5::OpenGL"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkGUISupportQtOpenGL-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkGUISupportQtOpenGL )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkGUISupportQtOpenGL "${_IMPORT_PREFIX}/lib/vtkGUISupportQtOpenGL-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkGUISupportQtOpenGL-7.0.dll" )

# Import target "vtksqlite" for configuration "Release"
set_property(TARGET vtksqlite APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtksqlite PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/vtksqlite-7.0.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtksqlite )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtksqlite "${_IMPORT_PREFIX}/lib/vtksqlite-7.0.lib" )

# Import target "vtkIOSQL" for configuration "Release"
set_property(TARGET vtkIOSQL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOSQL PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOSQL-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOSQL-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOSQL )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOSQL "${_IMPORT_PREFIX}/lib/vtkIOSQL-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOSQL-7.0.dll" )

# Import target "vtkGUISupportQtSQL" for configuration "Release"
set_property(TARGET vtkGUISupportQtSQL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGUISupportQtSQL PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkGUISupportQtSQL-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys;Qt5::Widgets;Qt5::Sql"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkGUISupportQtSQL-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkGUISupportQtSQL )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkGUISupportQtSQL "${_IMPORT_PREFIX}/lib/vtkGUISupportQtSQL-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkGUISupportQtSQL-7.0.dll" )

# Import target "vtkInfovisLayout" for configuration "Release"
set_property(TARGET vtkInfovisLayout APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkInfovisLayout PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkInfovisLayout-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkInfovisLayout-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkInfovisLayout )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkInfovisLayout "${_IMPORT_PREFIX}/lib/vtkInfovisLayout-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkInfovisLayout-7.0.dll" )

# Import target "vtkImagingColor" for configuration "Release"
set_property(TARGET vtkImagingColor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingColor PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingColor-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingColor-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingColor )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingColor "${_IMPORT_PREFIX}/lib/vtkImagingColor-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingColor-7.0.dll" )

# Import target "vtkRenderingAnnotation" for configuration "Release"
set_property(TARGET vtkRenderingAnnotation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingAnnotation PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingAnnotation-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersSources"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingAnnotation-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingAnnotation )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingAnnotation "${_IMPORT_PREFIX}/lib/vtkRenderingAnnotation-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingAnnotation-7.0.dll" )

# Import target "vtkRenderingVolume" for configuration "Release"
set_property(TARGET vtkRenderingVolume APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingVolume PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingVolume-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingVolume-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingVolume )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingVolume "${_IMPORT_PREFIX}/lib/vtkRenderingVolume-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingVolume-7.0.dll" )

# Import target "vtkInteractionWidgets" for configuration "Release"
set_property(TARGET vtkInteractionWidgets APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkInteractionWidgets PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkInteractionWidgets-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkInteractionWidgets-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkInteractionWidgets )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkInteractionWidgets "${_IMPORT_PREFIX}/lib/vtkInteractionWidgets-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkInteractionWidgets-7.0.dll" )

# Import target "vtkViewsCore" for configuration "Release"
set_property(TARGET vtkViewsCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkViewsCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkViewsCore-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkViewsCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkViewsCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkViewsCore "${_IMPORT_PREFIX}/lib/vtkViewsCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkViewsCore-7.0.dll" )

# Import target "vtkproj4" for configuration "Release"
set_property(TARGET vtkproj4 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkproj4 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkproj4-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkproj4-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkproj4 )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkproj4 "${_IMPORT_PREFIX}/lib/vtkproj4-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkproj4-7.0.dll" )

# Import target "vtkGeovisCore" for configuration "Release"
set_property(TARGET vtkGeovisCore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkGeovisCore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkGeovisCore-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkGeovisCore-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkGeovisCore )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkGeovisCore "${_IMPORT_PREFIX}/lib/vtkGeovisCore-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkGeovisCore-7.0.dll" )

# Import target "vtkhdf5" for configuration "Release"
set_property(TARGET vtkhdf5 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkhdf5 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkhdf5-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkhdf5-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkhdf5 )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkhdf5 "${_IMPORT_PREFIX}/lib/vtkhdf5-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkhdf5-7.0.dll" )

# Import target "vtkhdf5_hl" for configuration "Release"
set_property(TARGET vtkhdf5_hl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkhdf5_hl PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkhdf5_hl-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkhdf5_hl-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkhdf5_hl )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkhdf5_hl "${_IMPORT_PREFIX}/lib/vtkhdf5_hl-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkhdf5_hl-7.0.dll" )

# Import target "vtkIOAMR" for configuration "Release"
set_property(TARGET vtkIOAMR APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOAMR PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOAMR-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkhdf5_hl;vtkhdf5;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOAMR-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOAMR )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOAMR "${_IMPORT_PREFIX}/lib/vtkIOAMR-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOAMR-7.0.dll" )

# Import target "vtkIOEnSight" for configuration "Release"
set_property(TARGET vtkIOEnSight APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOEnSight PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOEnSight-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOEnSight-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOEnSight )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOEnSight "${_IMPORT_PREFIX}/lib/vtkIOEnSight-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOEnSight-7.0.dll" )

# Import target "vtkNetCDF" for configuration "Release"
set_property(TARGET vtkNetCDF APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkNetCDF PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkNetCDF-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkNetCDF-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkNetCDF )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkNetCDF "${_IMPORT_PREFIX}/lib/vtkNetCDF-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkNetCDF-7.0.dll" )

# Import target "vtkNetCDF_cxx" for configuration "Release"
set_property(TARGET vtkNetCDF_cxx APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkNetCDF_cxx PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkNetCDF_cxx-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkNetCDF_cxx-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkNetCDF_cxx )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkNetCDF_cxx "${_IMPORT_PREFIX}/lib/vtkNetCDF_cxx-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkNetCDF_cxx-7.0.dll" )

# Import target "vtkexoIIc" for configuration "Release"
set_property(TARGET vtkexoIIc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkexoIIc PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkexoIIc-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkexoIIc-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkexoIIc )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkexoIIc "${_IMPORT_PREFIX}/lib/vtkexoIIc-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkexoIIc-7.0.dll" )

# Import target "vtkIOExodus" for configuration "Release"
set_property(TARGET vtkIOExodus APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOExodus PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOExodus-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkexoIIc;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOExodus-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOExodus )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOExodus "${_IMPORT_PREFIX}/lib/vtkIOExodus-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOExodus-7.0.dll" )

# Import target "vtkRenderingContextOpenGL" for configuration "Release"
set_property(TARGET vtkRenderingContextOpenGL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingContextOpenGL PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingContextOpenGL-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkRenderingFreeType"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingContextOpenGL-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingContextOpenGL )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingContextOpenGL "${_IMPORT_PREFIX}/lib/vtkRenderingContextOpenGL-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingContextOpenGL-7.0.dll" )

# Import target "vtkgl2ps" for configuration "Release"
set_property(TARGET vtkgl2ps APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkgl2ps PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkgl2ps-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkgl2ps-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkgl2ps )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkgl2ps "${_IMPORT_PREFIX}/lib/vtkgl2ps-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkgl2ps-7.0.dll" )

# Import target "vtkRenderingGL2PS" for configuration "Release"
set_property(TARGET vtkRenderingGL2PS APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingGL2PS PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingGL2PS-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkRenderingOpenGL;vtkRenderingFreeType;vtkgl2ps"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingGL2PS-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingGL2PS )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingGL2PS "${_IMPORT_PREFIX}/lib/vtkRenderingGL2PS-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingGL2PS-7.0.dll" )

# Import target "vtkRenderingLabel" for configuration "Release"
set_property(TARGET vtkRenderingLabel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingLabel PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingLabel-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersExtraction"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingLabel-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingLabel )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingLabel "${_IMPORT_PREFIX}/lib/vtkRenderingLabel-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingLabel-7.0.dll" )

# Import target "vtkIOExport" for configuration "Release"
set_property(TARGET vtkIOExport APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOExport PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOExport-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkIOImage;vtkFiltersGeometry;vtkgl2ps"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOExport-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOExport )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOExport "${_IMPORT_PREFIX}/lib/vtkIOExport-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOExport-7.0.dll" )

# Import target "vtkIOImport" for configuration "Release"
set_property(TARGET vtkIOImport APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOImport PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOImport-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersSources;vtkIOImage"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOImport-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOImport )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOImport "${_IMPORT_PREFIX}/lib/vtkIOImport-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOImport-7.0.dll" )

# Import target "vtklibxml2" for configuration "Release"
set_property(TARGET vtklibxml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtklibxml2 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtklibxml2-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtklibxml2-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtklibxml2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtklibxml2 "${_IMPORT_PREFIX}/lib/vtklibxml2-7.0.lib" "${_IMPORT_PREFIX}/bin/vtklibxml2-7.0.dll" )

# Import target "vtkIOInfovis" for configuration "Release"
set_property(TARGET vtkIOInfovis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOInfovis PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOInfovis-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtklibxml2;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOInfovis-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOInfovis )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOInfovis "${_IMPORT_PREFIX}/lib/vtkIOInfovis-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOInfovis-7.0.dll" )

# Import target "vtkIOLSDyna" for configuration "Release"
set_property(TARGET vtkIOLSDyna APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOLSDyna PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOLSDyna-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOLSDyna-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOLSDyna )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOLSDyna "${_IMPORT_PREFIX}/lib/vtkIOLSDyna-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOLSDyna-7.0.dll" )

# Import target "vtkIOMINC" for configuration "Release"
set_property(TARGET vtkIOMINC APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOMINC PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOMINC-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys;vtkNetCDF;vtkNetCDF_cxx"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOMINC-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOMINC )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOMINC "${_IMPORT_PREFIX}/lib/vtkIOMINC-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOMINC-7.0.dll" )

# Import target "vtkoggtheora" for configuration "Release"
set_property(TARGET vtkoggtheora APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkoggtheora PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkoggtheora-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkoggtheora-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkoggtheora )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkoggtheora "${_IMPORT_PREFIX}/lib/vtkoggtheora-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkoggtheora-7.0.dll" )

# Import target "vtkIOMovie" for configuration "Release"
set_property(TARGET vtkIOMovie APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOMovie PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOMovie-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOMovie-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOMovie )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOMovie "${_IMPORT_PREFIX}/lib/vtkIOMovie-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOMovie-7.0.dll" )

# Import target "vtkIONetCDF" for configuration "Release"
set_property(TARGET vtkIONetCDF APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIONetCDF PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIONetCDF-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys;vtkNetCDF;vtkNetCDF_cxx"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIONetCDF-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIONetCDF )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIONetCDF "${_IMPORT_PREFIX}/lib/vtkIONetCDF-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIONetCDF-7.0.dll" )

# Import target "vtkIOPLY" for configuration "Release"
set_property(TARGET vtkIOPLY APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOPLY PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOPLY-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOPLY-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOPLY )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOPLY "${_IMPORT_PREFIX}/lib/vtkIOPLY-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOPLY-7.0.dll" )

# Import target "vtkjsoncpp" for configuration "Release"
set_property(TARGET vtkjsoncpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkjsoncpp PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkjsoncpp-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkjsoncpp-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkjsoncpp )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkjsoncpp "${_IMPORT_PREFIX}/lib/vtkjsoncpp-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkjsoncpp-7.0.dll" )

# Import target "vtkIOParallel" for configuration "Release"
set_property(TARGET vtkIOParallel APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOParallel PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOParallel-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkexoIIc;vtkjsoncpp;vtkNetCDF;vtkNetCDF_cxx;vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOParallel-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOParallel )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOParallel "${_IMPORT_PREFIX}/lib/vtkIOParallel-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOParallel-7.0.dll" )

# Import target "vtkIOParallelXML" for configuration "Release"
set_property(TARGET vtkIOParallelXML APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOParallelXML PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOParallelXML-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOParallelXML-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOParallelXML )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOParallelXML "${_IMPORT_PREFIX}/lib/vtkIOParallelXML-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOParallelXML-7.0.dll" )

# Import target "vtkIOVideo" for configuration "Release"
set_property(TARGET vtkIOVideo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkIOVideo PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkIOVideo-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkIOVideo-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkIOVideo )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkIOVideo "${_IMPORT_PREFIX}/lib/vtkIOVideo-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkIOVideo-7.0.dll" )

# Import target "vtkImagingMath" for configuration "Release"
set_property(TARGET vtkImagingMath APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingMath PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingMath-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingMath-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingMath )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingMath "${_IMPORT_PREFIX}/lib/vtkImagingMath-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingMath-7.0.dll" )

# Import target "vtkImagingMorphological" for configuration "Release"
set_property(TARGET vtkImagingMorphological APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingMorphological PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingMorphological-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingMorphological-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingMorphological )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingMorphological "${_IMPORT_PREFIX}/lib/vtkImagingMorphological-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingMorphological-7.0.dll" )

# Import target "vtkImagingStatistics" for configuration "Release"
set_property(TARGET vtkImagingStatistics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingStatistics PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingStatistics-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingStatistics-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingStatistics )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingStatistics "${_IMPORT_PREFIX}/lib/vtkImagingStatistics-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingStatistics-7.0.dll" )

# Import target "vtkImagingStencil" for configuration "Release"
set_property(TARGET vtkImagingStencil APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkImagingStencil PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkImagingStencil-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkImagingStencil-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkImagingStencil )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkImagingStencil "${_IMPORT_PREFIX}/lib/vtkImagingStencil-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkImagingStencil-7.0.dll" )

# Import target "vtkInteractionImage" for configuration "Release"
set_property(TARGET vtkInteractionImage APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkInteractionImage PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkInteractionImage-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkInteractionImage-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkInteractionImage )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkInteractionImage "${_IMPORT_PREFIX}/lib/vtkInteractionImage-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkInteractionImage-7.0.dll" )

# Import target "vtkRenderingImage" for configuration "Release"
set_property(TARGET vtkRenderingImage APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingImage PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingImage-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingImage-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingImage )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingImage "${_IMPORT_PREFIX}/lib/vtkRenderingImage-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingImage-7.0.dll" )

# Import target "vtkRenderingLIC" for configuration "Release"
set_property(TARGET vtkRenderingLIC APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingLIC PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingLIC-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingLIC-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingLIC )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingLIC "${_IMPORT_PREFIX}/lib/vtkRenderingLIC-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingLIC-7.0.dll" )

# Import target "vtkRenderingLOD" for configuration "Release"
set_property(TARGET vtkRenderingLOD APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingLOD PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingLOD-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingLOD-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingLOD )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingLOD "${_IMPORT_PREFIX}/lib/vtkRenderingLOD-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingLOD-7.0.dll" )

# Import target "vtkRenderingQt" for configuration "Release"
set_property(TARGET vtkRenderingQt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingQt PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingQt-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersTexture;vtkFiltersSources;vtkGUISupportQt;Qt5::Widgets"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingQt-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingQt )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingQt "${_IMPORT_PREFIX}/lib/vtkRenderingQt-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingQt-7.0.dll" )

# Import target "vtkRenderingVolumeOpenGL" for configuration "Release"
set_property(TARGET vtkRenderingVolumeOpenGL APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkRenderingVolumeOpenGL PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkRenderingVolumeOpenGL-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtksys;vtkFiltersGeneral;vtkFiltersSources"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkRenderingVolumeOpenGL-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkRenderingVolumeOpenGL )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkRenderingVolumeOpenGL "${_IMPORT_PREFIX}/lib/vtkRenderingVolumeOpenGL-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkRenderingVolumeOpenGL-7.0.dll" )

# Import target "vtkViewsContext2D" for configuration "Release"
set_property(TARGET vtkViewsContext2D APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkViewsContext2D PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkViewsContext2D-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkViewsContext2D-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkViewsContext2D )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkViewsContext2D "${_IMPORT_PREFIX}/lib/vtkViewsContext2D-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkViewsContext2D-7.0.dll" )

# Import target "vtkViewsInfovis" for configuration "Release"
set_property(TARGET vtkViewsInfovis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkViewsInfovis PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkViewsInfovis-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "vtkFiltersGeometry"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkViewsInfovis-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkViewsInfovis )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkViewsInfovis "${_IMPORT_PREFIX}/lib/vtkViewsInfovis-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkViewsInfovis-7.0.dll" )

# Import target "vtkViewsGeovis" for configuration "Release"
set_property(TARGET vtkViewsGeovis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkViewsGeovis PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkViewsGeovis-7.0.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkViewsGeovis-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkViewsGeovis )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkViewsGeovis "${_IMPORT_PREFIX}/lib/vtkViewsGeovis-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkViewsGeovis-7.0.dll" )

# Import target "vtkViewsQt" for configuration "Release"
set_property(TARGET vtkViewsQt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkViewsQt PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkViewsQt-7.0.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "Qt5::Widgets"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkViewsQt-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkViewsQt )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkViewsQt "${_IMPORT_PREFIX}/lib/vtkViewsQt-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkViewsQt-7.0.dll" )

# Import target "vtkLocalExample" for configuration "Release"
set_property(TARGET vtkLocalExample APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vtkLocalExample PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/vtkLocalExample-7.0.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "vtkCommonCore"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/vtkLocalExample-7.0.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS vtkLocalExample )
list(APPEND _IMPORT_CHECK_FILES_FOR_vtkLocalExample "${_IMPORT_PREFIX}/lib/vtkLocalExample-7.0.lib" "${_IMPORT_PREFIX}/bin/vtkLocalExample-7.0.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
