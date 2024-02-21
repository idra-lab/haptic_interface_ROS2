
#ifndef RaptorAPI_EXPORT_H
#define RaptorAPI_EXPORT_H

#ifdef RaptorAPI_BUILT_AS_STATIC
#  define RaptorAPI_EXPORT
#  define RAPTORAPI_NO_EXPORT
#else
#  ifndef RaptorAPI_EXPORT
#    ifdef RaptorAPI_lib_EXPORTS
        /* We are building this library */
#      define RaptorAPI_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define RaptorAPI_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef RAPTORAPI_NO_EXPORT
#    define RAPTORAPI_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef RAPTORAPI_DEPRECATED
#  define RAPTORAPI_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef RAPTORAPI_DEPRECATED_EXPORT
#  define RAPTORAPI_DEPRECATED_EXPORT RaptorAPI_EXPORT RAPTORAPI_DEPRECATED
#endif

#ifndef RAPTORAPI_DEPRECATED_NO_EXPORT
#  define RAPTORAPI_DEPRECATED_NO_EXPORT RAPTORAPI_NO_EXPORT RAPTORAPI_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef RAPTORAPI_NO_DEPRECATED
#    define RAPTORAPI_NO_DEPRECATED
#  endif
#endif

#endif /* RaptorAPI_EXPORT_H */
