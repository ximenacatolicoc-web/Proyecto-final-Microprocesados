# Monitor Biomédico Embebido con ESP32 (`ESP32_Pulse_Sensor`)

Prototipo de monitoreo biomédico basado en **ESP32** desarrollado en **PlatformIO** bajo una arquitectura no bloqueante (`millis()`). Adquiere señales fotopletismográficas (PPG) mediante el sensor **MAX30102** para estimar frecuencia cardíaca (BPM) y saturación periférica de oxígeno (SpO_2), detecta vibraciones con el **SW-420**, muestra información en tiempo real en una pantalla **OLED SSD1306**, activa una alarma sonora única (`SPO2_BAJO`), registra datos en formato `.csv` en una **MicroSD** y expone una interfaz web local en modo **Access Point**.

## Características principales
- **Adquisición y procesamiento PPG (`MAX30102`):** separación de componentes $\text{DC}$ y $\text{AC}$, estimación de $\text{SpO}_2$ por ventanas y detección de picos con promedio móvil de $\text{BPM}$.
- **Detección de vibración (`SW-420`):** lectura digital con retención temporal no bloqueante (`600 ms`) para estabilizar eventos mecánicos.
- **Alarma única (`BUZZER`):** gestión no bloqueante de alerta `SPO2_BAJO` (respetando presencia de dedo, validez de SpO2, umbral configurado y silenciamiento) y prueba manual acotada en tiempo.
- **Visualización local (`OLED SSD1306`):** refresco periódico con estado de MicroSD, vibración, aviso de colocación de dedo y lecturas de `FC` y `SpO2`.
- **Registro persistente (`MicroSD`):** exportación periódica a `/registro.csv` con marca de tiempo, muestras ópticas, señal filtrada, `BPM`, `SpO2`, umbral, vibración y motivo de alerta.
- **Interfaz web local (Modo AP):** punto de acceso `ESP32_Pulse_Sensor` accesible en `http://192.168.4.1` para consulta de datos y configuración.

## Arquitectura del sistema
El firmware ejecuta tareas periódicas sin bloqueos usando `millis()`:
1. Adquisición y procesamiento de muestras ópticas (`processSample()`, `updateSpO2Estimate()`).
2. Supervisión de vibración (`updateSw420()`) y control de alarma (`evaluateSingleAlert()`, `updateBuzzer()`).
3. Actualización de interfaz local, registro y diagnóstico (`updateOled()`, `updateSdLog()`, `updateSerial()`).


## Requisitos
### Hardware
- Placa **ESP32** (`esp32dev`)
- Sensor óptico **MAX30102**
- Sensor de vibración **SW-420**
- Pantalla **OLED SSD1306** (`0x3C`)
- **Buzzer**
- Módulo **MicroSD** (interfaz SPI) y tarjeta MicroSD

### Software
- **Visual Studio Code** con extensión **PlatformIO**
- Librerías configuradas en `platformio.ini`
  - sparkfun/SparkFun MAX3010x Pulse and Proximity Sensor Library
  - adafruit/Adafruit SSD1306
  - adafruit/Adafruit GFX Library

## Conexiones y red local
Consulta el detalle completo en [hardware/pinout.md](hardware/pinout.md).

- **SSID Access Point:** `ESP32_Pulse_Sensor`
- **URL interfaz local:** `http://192.168.4.1`
- **Archivo de registro:** `/registro.csv` en la MicroSD
  
