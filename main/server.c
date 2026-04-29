#include <esp_http_server.h>
#include <esp_log.h>

#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

static const char *TAG = "HTTP_SERVER";
static httpd_handle_t server = NULL;

// variables for auto stop
uint64_t last_request_time = 0; 
const uint64_t INACTIVITY_TIMEOUT = 5 * 60 * 1000000ULL;


// verifies sd card path
static bool is_valid_sdcard_path(const char *path) {
    if (path == NULL) {
        return false;
    }
    if (strncmp(path, "/sdcard", 7) != 0) {
        return false;
    }
    if (!(path[7] == '\0' || path[7] == '/')) {
        return false;
    }
    if (strstr(path, "..") != NULL) {
        return false;
    }
    return true;
}

static void format_file_size(off_t size, char *buffer, size_t buffer_size) {
    const char *units[] = {"B", "KB", "MB", "GB"};
    double scaled_size = (double)size;
    size_t unit_index = 0;

    while (scaled_size >= 1024.0 && unit_index < (sizeof(units) / sizeof(units[0])) - 1) {
        scaled_size /= 1024.0;
        unit_index++;
    }

    if (unit_index == 0) {
        snprintf(buffer, buffer_size, "%lld %s", (long long)size, units[unit_index]);
    } else {
        snprintf(buffer, buffer_size, "%.1f %s", scaled_size, units[unit_index]);
    }
}

// status handler
static esp_err_t status_get_handler(httpd_req_t *req) {
    const char* resp_str = "{\"status\": \"active\", \"msg\": \"data logger online\"}";
    httpd_resp_set_type(req, "application/json");
    last_request_time = esp_timer_get_time();

    return httpd_resp_send(req, resp_str, strlen(resp_str));
}

// file view handler
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

    if (!is_valid_sdcard_path(path)) {
        ESP_LOGE(TAG, "Invalid view path: %s", path);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid path");
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

        struct stat st;
        bool has_stat = stat(child_path, &st) == 0;

        // Check file type
        bool is_dir = false;
        if (content->d_type == DT_DIR) {
            is_dir = true;
        } else if (content->d_type == DT_UNKNOWN) {
            if (has_stat) {
                if (S_ISDIR(st.st_mode)) is_dir = true;
            }
        }

        // generate html
        if (is_dir) {
            snprintf(line, sizeof(line),
                "<li><strong>[DIR]</strong> <a href=\"/api/view?path=%s\">%s/</a></li>",
                child_path, content->d_name);
        } else {
            char file_size[32];
            if (has_stat) {
                format_file_size(st.st_size, file_size, sizeof(file_size));
            } else {
                snprintf(file_size, sizeof(file_size), "size unavailable");
            }

            snprintf(line, sizeof(line), 
                "<li><a href=\"/api/download?file=%s\">%s</a> (%s)</li>",
                child_path, content->d_name, file_size);
        }
        httpd_resp_sendstr_chunk(req, line);
    }

    httpd_resp_sendstr_chunk(req, "</ul></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    closedir(dir);

    last_request_time = esp_timer_get_time();

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
    if (httpd_query_key_value(full_req, "file", filename, sizeof(filename)) != ESP_OK) {
        ESP_LOGE(TAG, "Missing 'file' query parameter");
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing file parameter");
    }

    char path[512];
    if (filename[0] == '/') {
        strncpy(path, filename, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';
    } else {
        snprintf(path, sizeof(path), "/sdcard/%s", filename);
    }

    if (!is_valid_sdcard_path(path)) {
        ESP_LOGE(TAG, "Invalid download path: %s", path);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file path");
    }

    // open files for reading
    FILE *file = fopen(path, "rb");

    if(file == NULL) {
        ESP_LOGE(TAG, "file not found");
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "file not found");
    }

    // update last request time
    last_request_time = esp_timer_get_time();

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
        last_request_time = esp_timer_get_time();

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
    if(server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // registers handlers
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &file_view_uri);
        httpd_register_uri_handler(server,&file_download_uri);

        last_request_time = esp_timer_get_time();

        return ESP_OK;
    }
    return ESP_FAIL;
}

// stops server
void http_server_stop(void) {
    if (!server) {
        return;
    }

    else {
        httpd_stop(server);
        server = NULL;
    }
}

// auto stops after 5 mins of inactivity
void auto_server_stop() {
    if (!server) {
        return;
    }

    uint64_t now = esp_timer_get_time();

    if((now - last_request_time) >= INACTIVITY_TIMEOUT) {
        ESP_LOGI(TAG, "Stopping server due to inactivity.");
        http_server_stop();
    }
}

bool http_server_is_running() {
    return server != NULL;
}
