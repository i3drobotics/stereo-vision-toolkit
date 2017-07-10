set(vtkIOImage_LOADED 1)
set(vtkIOImage_DEPENDS "vtkCommonDataModel;vtkCommonExecutionModel;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkDICOMParser;vtkIOCore;vtkMetaIO;vtkjpeg;vtkpng;vtksys;vtktiff;vtkzlib")
set(vtkIOImage_LIBRARIES "vtkIOImage")
set(vtkIOImage_INCLUDE_DIRS "${VTK_INSTALL_PREFIX}/include/vtk-7.0")
set(vtkIOImage_LIBRARY_DIRS "")
set(vtkIOImage_RUNTIME_LIBRARY_DIRS "${VTK_INSTALL_PREFIX}/bin")
set(vtkIOImage_WRAP_HIERARCHY_FILE "${CMAKE_CURRENT_LIST_DIR}/vtkIOImageHierarchy.txt")

