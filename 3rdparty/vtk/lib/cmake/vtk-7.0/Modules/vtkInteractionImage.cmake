set(vtkInteractionImage_LOADED 1)
set(vtkInteractionImage_DEPENDS "vtkImagingColor;vtkInteractionStyle;vtkInteractionWidgets;vtkRenderingCore;vtkRenderingFreeType")
set(vtkInteractionImage_LIBRARIES "vtkInteractionImage")
set(vtkInteractionImage_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkInteractionImage_LIBRARY_DIRS "")
set(vtkInteractionImage_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkInteractionImage_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkInteractionImageHierarchy.txt")

