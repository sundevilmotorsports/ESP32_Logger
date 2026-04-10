#ifndef SERVER_H
#define SERVER_H


#include <esp_http_server.h>
#include <esp_log.h>
#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

// server start
esp_err_t http_server_start(void);

// server stop
void http_server_stop(void);

// server auto stop
void auto_server_stop();

// server running status
bool http_server_is_running();

#endif
