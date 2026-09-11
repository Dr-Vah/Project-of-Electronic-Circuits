#pragma once
#include "esp_https_server.h"
/* Public CA is downloadable; server private key is embedded only in firmware. */
extern const unsigned char tls_cert_start[] asm("_binary_server_cert_pem_start");
extern const unsigned char tls_cert_end[] asm("_binary_server_cert_pem_end");
extern const unsigned char tls_key_start[] asm("_binary_server_key_pem_start");
extern const unsigned char tls_key_end[] asm("_binary_server_key_pem_end");
static inline httpd_ssl_config_t local_tls_config(int port,int ctrl,int clients,int priority) {
    httpd_ssl_config_t c=HTTPD_SSL_CONFIG_DEFAULT();
    c.servercert=tls_cert_start;c.servercert_len=tls_cert_end-tls_cert_start;
    c.prvtkey_pem=tls_key_start;c.prvtkey_len=tls_key_end-tls_key_start;
    c.port_secure=port;c.httpd.ctrl_port=ctrl;c.httpd.max_open_sockets=clients;
    c.httpd.lru_purge_enable=true; /* Reclaim idle browser keep-alive sockets. */
    c.httpd.task_priority=priority;c.httpd.recv_wait_timeout=1;c.httpd.send_wait_timeout=1;
    return c;
}
