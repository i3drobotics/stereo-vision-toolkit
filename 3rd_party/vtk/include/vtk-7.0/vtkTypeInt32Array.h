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
// .NAME vtkTypeInt32Array - dynamic, self-adjusting array of vtkTypeInt32
// .SECTION Description
// vtkTypeInt32Array is an array of values of type vtkTypeInt32.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type Int32 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeInt32Array_h
#define vtkTypeInt32Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkIntArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeInt32Array : public vtkIntArray
{
public:
  static vtkTypeInt32Array* New();
  vtkTypeMacro(vtkTypeInt32Array,vtkIntArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeInt32Array();
  ~vtkTypeInt32Array();

private:
  vtkTypeInt32Array(const vtkTypeInt32Array&);  // Not implemented.
  void operator=(const vtkTypeInt32Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
