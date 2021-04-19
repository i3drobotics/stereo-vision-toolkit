/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkRenderingContextOpenGLObjectFactory.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef __vtkRenderingContextOpenGLObjectFactory_h
#define __vtkRenderingContextOpenGLObjectFactory_h

#include "vtkRenderingContextOpenGLModule.h" // For export macro
#include "vtkObjectFactory.h"

class VTKRENDERINGCONTEXTOPENGL_EXPORT vtkRenderingContextOpenGLObjectFactory : public vtkObjectFactory
{
public:
  static vtkRenderingContextOpenGLObjectFactory * New();
  vtkTypeMacro(vtkRenderingContextOpenGLObjectFactory, vtkObjectFactory)

  const char * GetDescription() { return "vtkRenderingContextOpenGL factory overrides."; }

  const char * GetVTKSourceVersion();

  void PrintSelf(ostream &os, vtkIndent indent);

protected:
  vtkRenderingContextOpenGLObjectFactory();

private:
  vtkRenderingContextOpenGLObjectFactory(const vtkRenderingContextOpenGLObjectFactory&); // Not implemented
  void operator=(const vtkRenderingContextOpenGLObjectFactory&);      // Not implemented
};

#endif // __vtkRenderingContextOpenGLObjectFactory_h
