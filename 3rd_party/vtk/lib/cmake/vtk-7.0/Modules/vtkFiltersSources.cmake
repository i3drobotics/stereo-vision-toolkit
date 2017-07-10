set(vtkFiltersSources_LOADED 1)
set(vtkFiltersSources_DEPENDS "vtkCommonComputationalGeometry;vtkFiltersGeneral")
set(vtkFiltersSources_LIBRARIES "vtkFiltersSources")
set(vtkFiltersSources_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkFiltersSources_LIBRARY_DIRS "")
set(vtkFiltersSources_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkFiltersSources_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkFiltersSourcesHierarchy.txt")

