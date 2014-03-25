#include <linux/string.h>

#ifndef LOG_TAG
#define LOG_TAG "LOG"
#endif

#define DEBUG_SWITCH
#ifdef DEBUG_SWITCH
#define D(fmt, args...) printk(LOG_TAG"-DEBUG  "fmt, ##args);
#define E(fmt, args...) printk(LOG_TAG"-ERROR  "fmt, ##args);
#else
#define D(fmt, args...) do{}while(0);
#define E(fmt, args...) do{}while(0);
#endif
