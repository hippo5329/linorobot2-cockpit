#ifndef SYSLOG_H_
#define SYSLOG_H_

#include "config.h"

// Remote logging over UDP, in the RFC 5424 form the arcao/Syslog library sent
// ("<PRI>1 - host app - - - msg"), so the cockpit's receiver sees the same
// lines. Our own: it is one datagram per call, and the library it replaced
// also cost an LDF include-path workaround between this and the wifi library.
//
// Needs a radio (HAS_WIFI: the silicon, never a robot choice). Whether a robot
// logs is the env: `syslog_ip` names the sink, and nothing is sent while the
// radio is off or unconnected.

#ifndef LOG_EMERG
#define LOG_EMERG   0
#define LOG_ALERT   1
#define LOG_CRIT    2
#define LOG_ERR     3
#define LOG_WARNING 4
#define LOG_NOTICE  5
#define LOG_INFO    6
#define LOG_DEBUG   7
#endif
#ifndef LOG_KERN
#define LOG_KERN    (0 << 3)
#define LOG_USER    (1 << 3)
#endif

#if defined(HAS_WIFI)
void syslog(uint16_t priority, const char *fmt, ...);
// Reads the sink from the env partition. Separate from any constructor: the
// flash partition API is not usable during static initialisation.
void initSyslog(void);
#else
#define syslog(...)
#define initSyslog()
#endif

#endif
