set(vtkIOParallel_LOADED 1)
set(vtkIOParallel_DEPENDS "vtkFiltersParallel;vtkIOImage;vtkIONetCDF;vtkIOXML;vtkParallelCore;vtkexodusII;vtkjsoncpp;vtknetcdf;vtksys")
set(vtkIOParallel_LIBRARIES "vtkIOParallel")
set(vtkIOParallel_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkIOParallel_LIBRARY_DIRS "")
set(vtkIOParallel_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkIOParallel_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkIOParallelHierarchy.txt")

