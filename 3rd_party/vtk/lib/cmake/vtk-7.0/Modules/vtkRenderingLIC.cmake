set(vtkRenderingLIC_LOADED 1)
set(vtkRenderingLIC_DEPENDS "vtkIOLegacy;vtkIOXML;vtkImagingSources;vtkRenderingOpenGL;vtksys")
set(vtkRenderingLIC_LIBRARIES "vtkRenderingLIC")
set(vtkRenderingLIC_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkRenderingLIC_LIBRARY_DIRS "")
set(vtkRenderingLIC_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkRenderingLIC_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkRenderingLICHierarchy.txt")

