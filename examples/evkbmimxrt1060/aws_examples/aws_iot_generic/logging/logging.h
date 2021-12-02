/*
 * FreeRTOS STM32 Reference Integration
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef LOGGING_H
#define LOGGING_H

/* Standard Include. */
#include <stdio.h>


#include "fsl_debug_console.h"


#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

/* Get rid of extra C89 style parentheses generated by core FreeRTOS libraries */

#define REMOVE_PARENS(...) STR(OVE __VA_ARGS__)
#define OVE(...) OVE __VA_ARGS__
#define STR(...) STR_(__VA_ARGS__)
#define STR_(...) REM ## __VA_ARGS__
#define REMOVE

/* Generic logging macros */
#define SdkLog( level, ... )        do { PRINTF( __VA_ARGS__ ); } while( 0 )

#define LogAssert( ... )            do { SdkLog( "ASRT", __VA_ARGS__ ); } while( 0 )

#define LogSys( ... )               do { PRINTF( __VA_ARGS__ ); } while( 0 )

#define LogKernel( ... )            SdkLog( "KRN", __VA_ARGS__ )

#if !defined( LOG_LEVEL ) ||       \
    ( ( LOG_LEVEL != LOG_NONE ) && \
    ( LOG_LEVEL != LOG_ERROR ) &&  \
    ( LOG_LEVEL != LOG_WARN ) &&   \
    ( LOG_LEVEL != LOG_INFO ) &&   \
    ( LOG_LEVEL != LOG_DEBUG ) )

    #error "Please define LOG_LEVEL as either LOG_NONE, LOG_ERROR, LOG_WARN, LOG_INFO, or LOG_DEBUG."
#else

    #if ( LOG_LEVEL >= LOG_ERROR )
        #define LogError( ... )         SdkLog( "ERR", REMOVE_PARENS( __VA_ARGS__ ) )
    #else
        #define LogError( ... )
    #endif

    #if ( LOG_LEVEL >= LOG_WARN )
        #define LogWarn( ... )          SdkLog( "WRN", REMOVE_PARENS( __VA_ARGS__ ) )
    #else
        #define LogWarn( ... )
    #endif

    #if ( LOG_LEVEL >= LOG_INFO )
        #define LogInfo( ... )          SdkLog( "INF", REMOVE_PARENS( __VA_ARGS__ ) )
    #else
        #define LogInfo( ... )
    #endif

    #if ( LOG_LEVEL >= LOG_DEBUG )
        #define LogDebug( ... )         SdkLog( "DBG", REMOVE_PARENS( __VA_ARGS__ ) )
    #else
        #define LogDebug( ... )
    #endif
#endif

#endif /* LOGGING_H */