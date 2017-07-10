set(vtkViewsInfovis_LOADED 1)
set(vtkViewsInfovis_DEPENDS "vtkChartsCore;vtkCommonColor;vtkFiltersGeometry;vtkFiltersImaging;vtkFiltersModeling;vtkInfovisLayout;vtkInteractionStyle;vtkRenderingContext2D;vtkRenderingLabel;vtkViewsCore")
set(vtkViewsInfovis_LIBRARIES "vtkViewsInfovis")
set(vtkViewsInfovis_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkViewsInfovis_LIBRARY_DIRS "")
set(vtkViewsInfovis_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkViewsInfovis_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkViewsInfovisHierarchy.txt")

