set(vtkViewsGeovis_LOADED 1)
set(vtkViewsGeovis_DEPENDS "vtkGeovisCore;vtkViewsInfovis")
set(vtkViewsGeovis_LIBRARIES "vtkViewsGeovis")
set(vtkViewsGeovis_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkViewsGeovis_LIBRARY_DIRS "")
set(vtkViewsGeovis_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkViewsGeovis_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkViewsGeovisHierarchy.txt")

