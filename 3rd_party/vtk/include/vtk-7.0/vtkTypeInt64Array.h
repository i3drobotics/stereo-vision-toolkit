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
// .NAME vtkTypeInt64Array - dynamic, self-adjusting array of vtkTypeInt64
// .SECTION Description
// vtkTypeInt64Array is an array of values of type vtkTypeInt64.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type Int64 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeInt64Array_h
#define vtkTypeInt64Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkLongLongArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeInt64Array : public vtkLongLongArray
{
public:
  static vtkTypeInt64Array* New();
  vtkTypeMacro(vtkTypeInt64Array,vtkLongLongArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeInt64Array();
  ~vtkTypeInt64Array();

private:
  vtkTypeInt64Array(const vtkTypeInt64Array&);  // Not implemented.
  void operator=(const vtkTypeInt64Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
