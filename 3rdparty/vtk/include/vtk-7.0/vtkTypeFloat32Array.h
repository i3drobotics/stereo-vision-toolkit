/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTypedArray.h.in

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkTypeFloat32Array - dynamic, self-adjusting array of vtkTypeFloat32
// .SECTION Description
// vtkTypeFloat32Array is an array of values of type vtkTypeFloat32.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type Float32 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeFloat32Array_h
#define vtkTypeFloat32Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkFloatArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeFloat32Array : public vtkFloatArray
{
public:
  static vtkTypeFloat32Array* New();
  vtkTypeMacro(vtkTypeFloat32Array,vtkFloatArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeFloat32Array();
  ~vtkTypeFloat32Array();

private:
  vtkTypeFloat32Array(const vtkTypeFloat32Array&);  // Not implemented.
  void operator=(const vtkTypeFloat32Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
