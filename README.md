# Proyecto-SCE
## Tema: 
Diseño de una planta de control y monitoreo de temperatura implementada en un cuarto de secado.

## Objetivos generales

* Monitorear y controlar de forma remota la temperatura de un cuarto de secado mediante el uso de una planta de control.
* Modificar el setpoint de temperatura de forma remota desde Telegram.
* Realizar control PWM mediante el uso de un ventilador de 12V para variar la temperatura medida por la planta.
* Utilizar las herramientas aprendidas durante el curso de sistemas de control embebidos.
* Diseñar un controlador con un porcentaje de precisión aceptable.

## Herramientas utilizadas

### Matlab
- Identificación del sistema y obtencion de la funcion de transferencia.
- Diseño de controlador PID mediante la obtención de los parámetros Kp, Ki y Kd usando PID Tuner.

### Mqtt Explorer
- Servidor que permite almacenar la información actual de la planta. 
- Se almacena la temperatura de los dos sensores, la temperatura promedio y el setpoint.

### Node-Red
- Intermediario entre la planta física y el control del setpoint mediante Telegram.
- Comunicación directa con el servidor MQTT para consultar las temperaturas actuales de la planta.

### Arduino IDE
Se usó para programar el sistema embebido (ESP32):
- Establecer conexión con servidor MQTT para envío de datos.
- Recibir el setpoint de temperatura enviado por Telegram mediante comunicación serial con Node-Red. 

### Telegram
- Monitoreo de los datos mediante consultas al servidor MQTT.
- Cambios en el setpoint para configurar la planta de forma remota.

### Cool Term
- Registro y exportación de datos para el sistema en lazo abierto.

## Autores:
Juan Orellana 
* https://github.com/juandi30

Giovanny Romero 
* https://github.com/girovizu

Axcel Espinoza 
* https://github.com/Axcel17

