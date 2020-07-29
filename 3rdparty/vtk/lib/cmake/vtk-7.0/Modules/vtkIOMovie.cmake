set(vtkIOMovie_LOADED 1)
set(vtkIOMovie_DEPENDS "vtkCommonDataModel;vtkCommonExecutionModel;vtkCommonSystem;vtkIOCore;vtkoggtheora")
set(vtkIOMovie_LIBRARIES "vtkIOMovie")
set(vtkIOMovie_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkIOMovie_LIBRARY_DIRS "")
set(vtkIOMovie_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkIOMovie_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkIOMovieHierarchy.txt")

