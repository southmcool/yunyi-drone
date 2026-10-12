#include "util/boot_log.h"
#include <cstdarg>
#include <cstdio>

#if !defined(SIL_HOST)
#include <Arduino.h>
#endif

namespace
{

    void vlog(const char *tag, const char *module, const char *fmt, va_list ap)
    {
#if defined(SIL_HOST)
        std::fprintf(stderr, "[BOOT] [%s] %s ", tag, module);
        std::vfprintf(stderr, fmt, ap);
        std::fputc('\n', stderr);
#else
        char buf[192];
        vsnprintf(buf, sizeof(buf), fmt, ap);
        Serial.printf("[BOOT] [%s] %s %s\n", tag, module, buf);
#endif
    }

} // namespace

namespace bootlog
{

    void info(const char *m, const char *f, ...)
    {
        va_list ap;
        va_start(ap, f);
        vlog("·", m, f, ap);
        va_end(ap);
    }
    void warn(const char *m, const char *f, ...)
    {
        va_list ap;
        va_start(ap, f);
        vlog("⚠", m, f, ap);
        va_end(ap);
    }
    void error(const char *m, const char *f, ...)
    {
        va_list ap;
        va_start(ap, f);
        vlog("✗", m, f, ap);
        va_end(ap);
    }
    void success(const char *m, const char *f, ...)
    {
        va_list ap;
        va_start(ap, f);
        vlog("✓", m, f, ap);
        va_end(ap);
    }

} // namespace bootlog