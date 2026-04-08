#include "logger.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifndef LOGGER_MAX_COLUMNS
#define LOGGER_MAX_COLUMNS 64
#endif

#ifndef LOGGER_MAX_ROWS
#define LOGGER_MAX_ROWS 10000
#endif

#ifndef LOGGER_MAX_NAME_LEN
#define LOGGER_MAX_NAME_LEN 64
#endif

#ifndef LOGGER_MAX_VALUE_LEN
#define LOGGER_MAX_VALUE_LEN 128
#endif

typedef struct
{
    char name[LOGGER_MAX_NAME_LEN];
} logger_column_t;

typedef struct
{
    char values[LOGGER_MAX_COLUMNS][LOGGER_MAX_VALUE_LEN];
    bool used[LOGGER_MAX_COLUMNS];
} logger_row_t;

static FILE *logfile = NULL;
static char logfile_path[256] = {0};

static logger_column_t columns[LOGGER_MAX_COLUMNS];
static int column_count = 0;

static logger_row_t rows[LOGGER_MAX_ROWS];
static int row_count = 0;

static logger_row_t current_row;
static uint64_t last_log_us = 0;

static uint64_t logger_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (uint64_t)ts.tv_sec * 1000000ULL +
           (uint64_t)(ts.tv_nsec / 1000ULL);
}

static void logger_make_filename(char *buffer, size_t size)
{
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    snprintf(buffer, size,
             "%slog_%04d-%02d-%02d_%02d-%02d-%02d.csv",
             LOG_FOLDER,
             t->tm_year + 1900,
             t->tm_mon + 1,
             t->tm_mday,
             t->tm_hour,
             t->tm_min,
             t->tm_sec);
}

static int logger_find_column(const char *name)
{
    for (int i = 0; i < column_count; i++) {
        if (strcmp(columns[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static int logger_get_or_create_column(const char *name)
{
    int idx = logger_find_column(name);
    if (idx >= 0) {
        return idx;
    }

    if (column_count >= LOGGER_MAX_COLUMNS) {
        return -1;
    }

    snprintf(columns[column_count].name, LOGGER_MAX_NAME_LEN, "%s", name);
    column_count++;
    return column_count - 1;
}

static void logger_clear_current_row(void)
{
    memset(&current_row, 0, sizeof(current_row));
}

static void logger_rewrite_file(void)
{
    if (logfile_path[0] == '\0') {
        return;
    }

    FILE *f = fopen(logfile_path, "w");
    if (!f) {
        printf("LOGGER ERROR: could not rewrite file: %s\n", logfile_path);
        return;
    }

    /* Header */
    for (int c = 0; c < column_count; c++) {
        fprintf(f, "%s", columns[c].name);
        if (c < column_count - 1) {
            fputc(',', f);
        }
    }
    fputc('\n', f);

    /* Rows */
    for (int r = 0; r < row_count; r++) {
        for (int c = 0; c < column_count; c++) {
            if (rows[r].used[c]) {
                fprintf(f, "%s", rows[r].values[c]);
            }
            if (c < column_count - 1) {
                fputc(',', f);
            }
        }
        fputc('\n', f);
    }

    fflush(f);
    fclose(f);
}

static void logger_set_value_internal(const char *name, const char *value)
{
    if (!name || !value) {
        return;
    }

    int idx = logger_get_or_create_column(name);
    if (idx < 0) {
        return;
    }

    snprintf(current_row.values[idx], LOGGER_MAX_VALUE_LEN, "%s", value);
    current_row.used[idx] = true;
}

bool logger_is_ready(void)
{
    return (logfile_path[0] != '\0');
}

void logger_init(void)
{
    mkdir(LOG_FOLDER, 0777);

    logger_make_filename(logfile_path, sizeof(logfile_path));

    logfile = fopen(logfile_path, "w");
    if (!logfile) {
        printf("LOGGER ERROR: could not open file: %s\n", logfile_path);
        logfile_path[0] = '\0';
        return;
    }

    fclose(logfile);
    logfile = NULL;

    printf("Logging to: %s\n", logfile_path);

    column_count = 0;
    row_count = 0;
    last_log_us = 0;
    logger_clear_current_row();
}

void logger_close(void)
{
    logfile = NULL;
    logfile_path[0] = '\0';
    column_count = 0;
    row_count = 0;
    last_log_us = 0;
    logger_clear_current_row();
}

void logger_begin_row(void)
{
    logger_clear_current_row();
}

void logger_set_float(const char *name, float value)
{
    char buffer[LOGGER_MAX_VALUE_LEN];
    snprintf(buffer, sizeof(buffer), "%.6f", value);
    logger_set_value_internal(name, buffer);
}

void logger_set_double(const char *name, double value)
{
    char buffer[LOGGER_MAX_VALUE_LEN];
    snprintf(buffer, sizeof(buffer), "%.6f", value);
    logger_set_value_internal(name, buffer);
}

void logger_set_int(const char *name, int value)
{
    char buffer[LOGGER_MAX_VALUE_LEN];
    snprintf(buffer, sizeof(buffer), "%d", value);
    logger_set_value_internal(name, buffer);
}

void logger_set_string(const char *name, const char *value)
{
    logger_set_value_internal(name, value);
}

void logger_end_row(void)
{
    if (!logger_is_ready()) {
        return;
    }

    const uint64_t now_us = logger_time_us();
    const uint64_t interval_us = 1000000ULL / LOG_RATE_HZ;

    if ((now_us - last_log_us) < interval_us) {
        return;
    }

    last_log_us = now_us;

    if (row_count >= LOGGER_MAX_ROWS) {
        return;
    }

    rows[row_count] = current_row;
    row_count++;

    logger_rewrite_file();
}