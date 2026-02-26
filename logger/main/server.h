#ifndef SERVER_H
#define SERVER_H
#endif

#include <esp_http_server.h>
#include <esp_log.h>
#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "remote_access.h"

// server handlers
static esp_err_t status_get_handler(httpd_req_t *req);
static esp_err_t file_view_handler(httpd_req_t *req);
static esp_err_t file_download_handler(httpd_req_t *req);

// server start
esp_err_t http_server_start(void);

// server stop
void http_server_stop(void);