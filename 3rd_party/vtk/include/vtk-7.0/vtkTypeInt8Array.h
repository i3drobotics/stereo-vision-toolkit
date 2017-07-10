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
// .NAME vtkTypeInt8Array - dynamic, self-adjusting array of vtkTypeInt8
// .SECTION Description
// vtkTypeInt8Array is an array of values of type vtkTypeInt8.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type Int8 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeInt8Array_h
#define vtkTypeInt8Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkCharArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeInt8Array : public vtkCharArray
{
public:
  static vtkTypeInt8Array* New();
  vtkTypeMacro(vtkTypeInt8Array,vtkCharArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeInt8Array();
  ~vtkTypeInt8Array();

private:
  vtkTypeInt8Array(const vtkTypeInt8Array&);  // Not implemented.
  void operator=(const vtkTypeInt8Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
