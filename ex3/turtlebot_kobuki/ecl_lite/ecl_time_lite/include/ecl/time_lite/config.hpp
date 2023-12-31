/**
 * @file /include/ecl/time_lite/config.hpp
 * 
 * @brief Pre-processed macros that define time for the platform.
 * 
 * Cmake is used to check/verify your target platform parameters related
 * to time functions. At the moment this is all automated. If ever this
 * needs to change (i.e. if we need to hand configure some macros as per
 * ecl_config style) then we should move this to ecl_config itself.
 * 
 * @date February 2011
 **/
/*****************************************************************************
** Guards
*****************************************************************************/

#ifndef ECL_TIME_CONFIG_HPP_
#define ECL_TIME_CONFIG_HPP_

/*****************************************************************************
** Include
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/**
 * @addtogroup Macros
 * @{
**/

/*****************************************************************************
** Private macros
*****************************************************************************/
/**
 * @def ECL_PRIVATE_HAS_CLOCK_GETTIME
 * 
 * @brief The platform has clock_gettime() from -lrt.
**/
/* #undef ECL_PRIVATE_HAS_CLOCK_GETTIME */
#define ECL_PRIVATE_HAS_CLOCK_GETTIME
/**
 * @def ECL_PRIVATE_HAS_CLOCK_NANOSLEEP
 *
 * @brief The platform has clock_nanosleep() from -lrt.
**/
/* #undef ECL_PRIVATE_HAS_CLOCK_NANOSLEEP */
#define ECL_PRIVATE_HAS_CLOCK_NANOSLEEP

/*****************************************************************************
** Public macros
*****************************************************************************/
/**
 * @def ECL_HAS_CLOCK_MONOTONIC
 *
 * @brief Can utilise monotonic clocks.
 *
 * These come with -lrt and allow use of timers which are guaranteed not
 * to jump.
**/
/* #undef ECL_HAS_CLOCK_MONOTONIC */
/**
 * @def ECL_HAS_CPUTIME
 *
 * @brief Can set clocks measuring time spent on the cpu (-lrt).
**/
/* #undef ECL_HAS_CPUTIME */
#define ECL_HAS_CPUTIME

/**
 * @def ECL_HAS_MACH_TIMERS
 *
 * @brief Ecl uses the mac timers.
 * 
 * - ECL_HAS_WIN_TIMERS
 * - ECL_HAS_RT_TIMERS
 * - ECL_HAS_POSIX_TIMERS
**/
/* #undef ECL_HAS_MACH_TIMERS */
/**
 * @def ECL_HAS_WIN_TIMERS
 *
 * @brief Ecl uses the win timers.
 * 
 * - ECL_HAS_RT_TIMERS
 * - ECL_HAS_MACH_TIMERS
 * - ECL_HAS_POSIX_TIMERS
**/
/* #undef ECL_HAS_WIN_TIMERS */

/**
 * @def ECL_HAS_RT_TIMERS
 *
 * @brief Ecl uses the real time (-lrt) timers. 
 * 
 * This assumption was made by checking for the presence of clock_gettime,
 * and clock_nanosleep. See also:
 * 
 * - ECL_HAS_WIN_TIMERS
 * - ECL_HAS_MACH_TIMERS
 * - ECL_HAS_POSIX_TIMERS
**/
/* #undef ECL_HAS_RT_TIMERS */

/**
 * @def ECL_HAS_POSIX_TIMERS
 *
 * @brief Ecl uses the default posix timers.
 * 
 * - ECL_HAS_WIN_TIMERS
 * - ECL_HAS_MACH_TIMERS
 * - ECL_HAS_RT_TIMERS
**/
/* #undef ECL_HAS_POSIX_TIMERS */
#define ECL_HAS_POSIX_TIMERS
/**
 * @}
 **/

#endif /* ECL_TIME_CONFIG_HPP_ */
 
