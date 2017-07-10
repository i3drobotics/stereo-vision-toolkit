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
// .NAME vtkTypeInt16Array - dynamic, self-adjusting array of vtkTypeInt16
// .SECTION Description
// vtkTypeInt16Array is an array of values of type vtkTypeInt16.  It
// provides methods for insertion and retrieval of values and will
// automatically resize itself to hold new data.
//
// This array should be preferred for data of type Int16 as this
// array will use the correct underlying datatype based on the desired number of bits
// and the current platform.  The superclass of this type will change depending on the
// machine and compiler in use so that the data contained always uses the proper type.

#ifndef vtkTypeInt16Array_h
#define vtkTypeInt16Array_h

#include "vtkCommonCoreModule.h" // For export macro
#include "vtkShortArray.h"

class VTKCOMMONCORE_EXPORT vtkTypeInt16Array : public vtkShortArray
{
public:
  static vtkTypeInt16Array* New();
  vtkTypeMacro(vtkTypeInt16Array,vtkShortArray);
  void PrintSelf(ostream& os, vtkIndent indent);

protected:
  vtkTypeInt16Array();
  ~vtkTypeInt16Array();

private:
  vtkTypeInt16Array(const vtkTypeInt16Array&);  // Not implemented.
  void operator=(const vtkTypeInt16Array&);  // Not implemented.
};

#endif
// VTK-HeaderTest-Exclude: vtkTypedArray.h.in
