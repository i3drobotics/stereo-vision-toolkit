set(vtkFiltersCore_LOADED 1)
set(vtkFiltersCore_DEPENDS "vtkCommonExecutionModel;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms")
set(vtkFiltersCore_LIBRARIES "vtkFiltersCore")
set(vtkFiltersCore_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkFiltersCore_LIBRARY_DIRS "")
set(vtkFiltersCore_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkFiltersCore_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkFiltersCoreHierarchy.txt")

