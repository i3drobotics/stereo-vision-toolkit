
#ifndef VTKRENDERINGCONTEXTOPENGL_EXPORT_H
#define VTKRENDERINGCONTEXTOPENGL_EXPORT_H

#ifdef VTKRENDERINGCONTEXTOPENGL_STATIC_DEFINE
#  define VTKRENDERINGCONTEXTOPENGL_EXPORT
#  define VTKRENDERINGCONTEXTOPENGL_NO_EXPORT
#else
#  ifndef VTKRENDERINGCONTEXTOPENGL_EXPORT
#    ifdef vtkRenderingContextOpenGL_EXPORTS
        /* We are building this library */
#      define VTKRENDERINGCONTEXTOPENGL_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define VTKRENDERINGCONTEXTOPENGL_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef VTKRENDERINGCONTEXTOPENGL_NO_EXPORT
#    define VTKRENDERINGCONTEXTOPENGL_NO_EXPORT 
#  endif
#endif

#ifndef VTKRENDERINGCONTEXTOPENGL_DEPRECATED
#  define VTKRENDERINGCONTEXTOPENGL_DEPRECATED __declspec(deprecated)
#  define VTKRENDERINGCONTEXTOPENGL_DEPRECATED_EXPORT VTKRENDERINGCONTEXTOPENGL_EXPORT __declspec(deprecated)
#  define VTKRENDERINGCONTEXTOPENGL_DEPRECATED_NO_EXPORT VTKRENDERINGCONTEXTOPENGL_NO_EXPORT __declspec(deprecated)
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKRENDERINGCONTEXTOPENGL_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkRenderingContext2DModule.h"
#include "vtkRenderingOpenGLModule.h"

/* AutoInit implementations.  */
#if defined(vtkRenderingContextOpenGL_INCLUDE)
# include vtkRenderingContextOpenGL_INCLUDE
#endif
#if defined(vtkRenderingContextOpenGL_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkRenderingContextOpenGL)
#endif

#endif
