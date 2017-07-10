set(vtkGeovisCore_LOADED 1)
set(vtkGeovisCore_DEPENDS "vtkIOXML;vtkInfovisLayout;vtkInteractionStyle;vtkInteractionWidgets;vtkRenderingCore;vtkViewsCore;vtklibproj4")
set(vtkGeovisCore_LIBRARIES "vtkGeovisCore")
set(vtkGeovisCore_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkGeovisCore_LIBRARY_DIRS "")
set(vtkGeovisCore_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkGeovisCore_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkGeovisCoreHierarchy.txt")

