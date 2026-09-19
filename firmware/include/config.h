#ifndef CONFIG_H
#define CONFIG_H

#ifdef CONFIG_PATH
    #include CONFIG_PATH
#else
    #include "custom/lino_base_config.h"
#endif

#endif
