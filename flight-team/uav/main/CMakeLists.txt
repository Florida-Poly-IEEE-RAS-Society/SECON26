idf_component_register(SRCS
  "uav.c"
  "IR/ir.c"
  "Camera/camera.c"
  "server/server.c"
  "Gyro/bno055.c"
  "Wifi/wifi.c"
  "Motor/motor.c"
  INCLUDE_DIRS "."
  REQUIRES "esp_timer")
