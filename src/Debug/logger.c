#include "Debug/logger.h"
#include <stdio.h>

void logger_init(void) {}
void logger_close(void) {}
void logger_begin_row(void) {}
void logger_set_double(const char *name, double value) {(void)name; (void)value;}
void logger_set_float(const char *name, float value) {(void)name; (void)value;}
void logger_set_string(const char *name, const char *value) {(void)name; (void)value;}
void logger_end_row(void) {}
