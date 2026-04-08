#ifndef LOGGER_H
#define LOGGER_H

#include <stdbool.h>

void logger_init(void);
void logger_close(void);

void logger_begin_row(void);
void logger_set_float(const char *name, float value);
void logger_set_double(const char *name, double value);
void logger_set_int(const char *name, int value);
void logger_set_string(const char *name, const char *value);
void logger_end_row(void);

bool logger_is_ready(void);

#endif