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
// .NAME vtkTypeUInt64Array - dynamic, self-adjusting array of vtkTypeUInt64
// .SECTION Description
// vtkTypeUInt64Array is an array of values of type vtkTypeUInt64.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type UInt64 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeUInt64Array_h
#define vtkTypeUInt64Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkUnsignedLongLongArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeUInt64Array : public vtkUnsignedLongLongArray
{
public:
  static vtkTypeUInt64Array* New();
  vtkTypeMacro(vtkTypeUInt64Array,vtkUnsignedLongLongArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeUInt64Array();
  ~vtkTypeUInt64Array();

private:
  vtkTypeUInt64Array(const vtkTypeUInt64Array&);  // Not implemented.
  void operator=(const vtkTypeUInt64Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
