#include <esp_http_server.h>
#include <esp_log.h>
#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "remote_access.h"

static const char *TAG = "HTTP_SERVER";
static httpd_handle_t server = NULL;

// basic json response handler for now
static esp_err_t status_get_handler(httpd_req_t *req) {
    const char* resp_str = "{\"status\": \"active\", \"msg\": \"data logger online\"}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp_str, strlen(resp_str));
}

static esp_err_t file_view_handler(httpd_req_t *req) {
    char line[1024];
    char path[256];     
    char child_path[512];
    
    strcpy(path, "/sdcard");

    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0) {
        char query[512];
        httpd_req_get_url_query_str(req, query, sizeof(query));
        if (httpd_query_key_value(query, "path", path, sizeof(path)) != ESP_OK) {
            strcpy(path, "/sdcard");
        }
    }

    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open dir: %s", path);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open directory");
    }

    httpd_resp_sendstr_chunk(req, "<html><body><h1>Data Logger Explorer</h1><ul>");

    // back button
    if (strcmp(path, "/sdcard") != 0) {
        snprintf(line, sizeof(line), "<li><a href=\"/api/view?path=/sdcard\"><strong>.. (Back to Root)</strong></a></li>");
        httpd_resp_sendstr_chunk(req, line);
    }

    struct dirent *content;
    while ((content = readdir(dir)) != NULL) {
        if (content->d_name[0] == '.') continue;

        // build path
        if (strcmp(path, "/") == 0) {
            snprintf(child_path, sizeof(child_path), "/%s", content->d_name);
        } else {
            snprintf(child_path, sizeof(child_path), "%s/%s", path, content->d_name);
        }

        // Check file type
        bool is_dir = false;
        if (content->d_type == DT_DIR) {
            is_dir = true;
        } else if (content->d_type == DT_UNKNOWN) {
            struct stat st;
            if (stat(child_path, &st) == 0) {
                if (S_ISDIR(st.st_mode)) is_dir = true;
            }
        }

        // generate html
        if (is_dir) {
            snprintf(line, sizeof(line),
                "<li><strong>[DIR]</strong> <a href=\"/api/view?path=%s\">%s/</a></li>",
                child_path, content->d_name);
        } else {
            snprintf(line, sizeof(line), 
                "<li><a href=\"/api/download?file=%s\">%s</a></li>", 
                child_path, content->d_name);
        }
        httpd_resp_sendstr_chunk(req, line);
    }

    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    closedir(dir);
    return ESP_OK;
}

// download handler
static esp_err_t file_download_handler(httpd_req_t *req) {
    size_t req_size = httpd_req_get_url_query_len(req);

    if (req_size == 0) {
        ESP_LOGE(TAG, "No file selected for download");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No file selected");
    }

    else if (req_size + 1 > 128) {
        ESP_LOGE(TAG, "Request size too large");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Filename too long");
    }

    // finding filename
    char full_req[128];
    httpd_req_get_url_query_str(req, full_req, sizeof(full_req));

    char filename[128];
    httpd_query_key_value(full_req, "file", filename, sizeof(filename));

    char path[512];
    if (filename[0] == '/') {
        strncpy(path, filename, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
    } else {
        snprintf(path, sizeof(path), "/sdcard/%s", filename);
    }

    // open files for reading
    FILE *file = fopen(path, "rb");

    if(file == NULL) {
        ESP_LOGE(TAG, "file not found");
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "file not found");
    }

    // http headers
    httpd_resp_set_type(req, "application/octet-stream");
    // use basename for Content-Disposition
    char *base = strrchr(filename, '/');
    if (base) base++;
    else base = filename;
    char disposition[256];
    snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", base);
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);

    char buffer[512];
    int chunk_size = 0;

    // file reading
    while((chunk_size = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        if(httpd_resp_send_chunk(req, buffer, chunk_size) != ESP_OK) {
            ESP_LOGE(TAG, "Error sending file");
            fclose(file);
            return ESP_FAIL;
        }
    }

    fclose(file);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// handler routings
static const httpd_uri_t status_uri = {
    .uri       = "/api/status",
    .method    = HTTP_GET,
    .handler   = status_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t file_view_uri = {
    .uri = "/api/view",
    .method = HTTP_GET,
    .handler = file_view_handler,
    .user_ctx = NULL
};

static const httpd_uri_t file_download_uri = {
    .uri = "/api/download",
    .method = HTTP_GET,
    .handler = file_download_handler,
    .user_ctx = NULL
};

// manages server state 
esp_err_t http_server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // registers handlers
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &file_view_uri);
        httpd_register_uri_handler(server,&file_download_uri);
        return ESP_OK;
    }
    return ESP_FAIL;
}

// stops server
void http_server_stop(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
}