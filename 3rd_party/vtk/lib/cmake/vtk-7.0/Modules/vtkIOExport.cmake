set(vtkIOExport_LOADED 1)
set(vtkIOExport_DEPENDS "vtkCommonCore;vtkFiltersGeometry;vtkIOImage;vtkImagingCore;vtkRenderingAnnotation;vtkRenderingContext2D;vtkRenderingCore;vtkRenderingFreeType;vtkRenderingGL2PS;vtkRenderingLabel;vtkRenderingOpenGL;vtkgl2ps")
set(vtkIOExport_LIBRARIES "vtkIOExport")
set(vtkIOExport_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkIOExport_LIBRARY_DIRS "")
set(vtkIOExport_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkIOExport_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkIOExportHierarchy.txt")

