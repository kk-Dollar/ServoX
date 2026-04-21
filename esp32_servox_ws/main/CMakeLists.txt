idf_component_register(SRCS "app_main.c"
                            "pca9685_driver.c"
                            "servo_controller.c"
                            "safety_layer.c"
                            "microros_node.c"
                            "wifi_transport.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_timer
                                  nvs_flash
                                  esp_wifi
                                  esp_event
                                  esp_netif
                                  micro_ros_espidf_component)
