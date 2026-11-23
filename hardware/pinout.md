# Conexiones y alimentación del hardware

| Periférico / Bus | Pin / Señal | GPIO ESP32 | Observaciones |
| :--- | :--- | :--- | :--- |
| **MAX30102 (Bus 0 I2C)** | SDA / SCL | GPIO 21 / GPIO 22 | Sensor óptico PPG (FC y SpO2) |
| **OLED SSD1306 (Bus 1 I2C)** | SDA / SCL | GPIO 25 / GPIO 26 | Dirección I2C `0x3C` |
| **SW-420** | DO (Salida digital) | GPIO 32 | Detección de vibración con retención (`600 ms`) |
| **Buzzer** | Señal activa/control | GPIO 27 | Alarma única (`SPO2_BAJO` / prueba manual) |
| **MicroSD (SPI)** | CS / SCK / MOSI / MISO | GPIO 5 / 18 / 23 / 19 | Alimentar `VCC` desde `VIN` / `5V` según módulo |
