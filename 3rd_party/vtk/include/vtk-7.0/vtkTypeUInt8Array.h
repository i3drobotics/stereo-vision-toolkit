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
// .NAME vtkTypeUInt8Array - dynamic, self-adjusting array of vtkTypeUInt8
// .SECTION Description
// vtkTypeUInt8Array is an array of values of type vtkTypeUInt8.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type UInt8 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeUInt8Array_h
#define vtkTypeUInt8Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkUnsignedCharArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeUInt8Array : public vtkUnsignedCharArray
{
public:
  static vtkTypeUInt8Array* New();
  vtkTypeMacro(vtkTypeUInt8Array,vtkUnsignedCharArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeUInt8Array();
  ~vtkTypeUInt8Array();

private:
  vtkTypeUInt8Array(const vtkTypeUInt8Array&);  // Not implemented.
  void operator=(const vtkTypeUInt8Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
