#ifndef SYSLOG_H_
#define SYSLOG_H_ 

#include <Syslog.h>
#include "config.h"

// Needs the radio: syslog is UDP. USE_SYSLOG says the config wants
// remote logging; USE_WIFI says this image has something to send it over.
#if defined(USE_SYSLOG) && defined(USE_WIFI)
void syslog(uint16_t priority, const char *fmt, ...);
// Points syslogv at the server named in the env partition. Separate from the
// constructor on purpose: syslogv is a global, so it is built during static
// initialisation, which runs before the flash partition API is usable. Reading
// the env there would fault or silently return nothing.
void initSyslog(void);
#else
#define syslog(...)
#define initSyslog()
#endif

#endif
