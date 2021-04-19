set(vtkIOInfovis_LOADED 1)
set(vtkIOInfovis_DEPENDS "vtkCommonDataModel;vtkCommonMisc;vtkCommonSystem;vtkIOCore;vtkIOLegacy;vtkIOXML;vtkInfovisCore;vtklibxml2;vtksys")
set(vtkIOInfovis_LIBRARIES "vtkIOInfovis")
set(vtkIOInfovis_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkIOInfovis_LIBRARY_DIRS "")
set(vtkIOInfovis_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkIOInfovis_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkIOInfovisHierarchy.txt")

