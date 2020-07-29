set(vtkRenderingOpenGL_LOADED 1)
set(vtkRenderingOpenGL_DEPENDS "vtkImagingHybrid;vtkParseOGLExt;vtkRenderingCore;vtkUtilitiesEncodeString;vtksys")
set(vtkRenderingOpenGL_LIBRARIES "vtkRenderingOpenGL")
set(vtkRenderingOpenGL_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkRenderingOpenGL_LIBRARY_DIRS "")
set(vtkRenderingOpenGL_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkRenderingOpenGL_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkRenderingOpenGLHierarchy.txt")
set(vtkRenderingOpenGL_IMPLEMENTS "vtkRenderingCore")

