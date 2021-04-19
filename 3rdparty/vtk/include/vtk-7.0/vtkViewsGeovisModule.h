
#ifndef VTKVIEWSGEOVIS_EXPORT_H
#define VTKVIEWSGEOVIS_EXPORT_H

#ifdef VTKVIEWSGEOVIS_STATIC_DEFINE
#  define VTKVIEWSGEOVIS_EXPORT
#  define VTKVIEWSGEOVIS_NO_EXPORT
#else
#  ifndef VTKVIEWSGEOVIS_EXPORT
#    ifdef vtkViewsGeovis_EXPORTS
        /* We are building this library */
#      define VTKVIEWSGEOVIS_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define VTKVIEWSGEOVIS_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef VTKVIEWSGEOVIS_NO_EXPORT
#    define VTKVIEWSGEOVIS_NO_EXPORT 
#  endif
#endif

#ifndef VTKVIEWSGEOVIS_DEPRECATED
#  define VTKVIEWSGEOVIS_DEPRECATED __declspec(deprecated)
#  define VTKVIEWSGEOVIS_DEPRECATED_EXPORT VTKVIEWSGEOVIS_EXPORT __declspec(deprecated)
#  define VTKVIEWSGEOVIS_DEPRECATED_NO_EXPORT VTKVIEWSGEOVIS_NO_EXPORT __declspec(deprecated)
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKVIEWSGEOVIS_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkGeovisCoreModule.h"
#include "vtkViewsInfovisModule.h"

#endif
