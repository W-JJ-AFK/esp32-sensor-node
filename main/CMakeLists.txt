idf_component_register(
    SRCS "app_main.c"
         "algo/edge_detect.c"
         "algo/imu_ring.c"
         "drivers/dht22.c"
         "drivers/mpu6050.c"
         "drivers/ssd1306.c"
         "net/protocol.c"
         "net/topic.c"
         "net/time_sync.c"
         "net/wifi_mgr.c"
         "net/mqtt_mgr.c"
         "tasks/task_imu.c"
         "tasks/task_detect.c"
         "tasks/task_dht.c"
         "tasks/task_net.c"
         "tasks/task_display.c"
         "tasks/task_monitor.c"
    INCLUDE_DIRS "."
                 "algo"
                 "drivers"
                 "net"
                 "tasks"
    REQUIRES driver
             esp_wifi
             esp_netif
             lwip
             mqtt
             json
             esp_timer
             nvs_flash
)
