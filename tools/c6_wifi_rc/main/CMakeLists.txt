idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       EMBED_TXTFILES "web/index.html"
                       PRIV_REQUIRES esp_wifi esp_netif nvs_flash esp_http_server driver)
