set(vtkIOLegacy_LOADED 1)
set(vtkIOLegacy_DEPENDS "vtkCommonDataModel;vtkCommonMisc;vtkCommonSystem;vtkIOCore;vtksys")
set(vtkIOLegacy_LIBRARIES "vtkIOLegacy")
set(vtkIOLegacy_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkIOLegacy_LIBRARY_DIRS "")
set(vtkIOLegacy_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkIOLegacy_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkIOLegacyHierarchy.txt")

