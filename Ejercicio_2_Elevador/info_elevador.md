
---

# 🚪 Proyecto Elevador Automatizado — STM32 Nucleo-L053R8

**Autor:** Kevin González
**Curso:** Arquitectura de Computadoras y Microcontroladores
**Serie 1 – Proyecto Integrador**
**Plataforma:** Nucleo-64 STM32L053R8

---

## 🧠 Descripción General

Este proyecto implementa un **elevador automatizado funcional**, desarrollado íntegramente con la placa **STM32 Nucleo-L053R8**, utilizando **procesamiento paralelo mediante interrupciones**.
El objetivo fue **simular un sistema real** de control de elevador, cumpliendo con todos los requerimientos de uso de **módulos internos (Timers, GPIO, UART)** y **circuitería externa (LCD, Keypad, 7 segmentos, Buzzer, etc.)** sin utilizar retardos por software ni funciones `delay()`.

La maqueta es funcional con una **estructura impresa en 3D** y una **base de acrílico** que protege el circuito principal. Se adjunta el archivo .blend para ver el modelo en caso de que se quiera replicar el proyecto.

---

## ⚙️ Componentes y Funcionalidades

| Componente                                          | Descripción                                 | Uso en el sistema                                                                        |
| --------------------------------------------------- | ------------------------------------------- | ---------------------------------------------------------------------------------------- |
| 🧠 **STM32 Nucleo-L053R8**                          | Microcontrolador principal (ARM Cortex-M0+) | Control general por interrupciones                                                       |
| 🎛️ **Keypad 4×4**                                  | Se usan 3 columnas y 1 fila                 | Solicitud de piso desde dentro del elevador                                              |
| 🔘 **Push Buttons (con circuito debounce físico)**  | Tres botones externos                       | Solicitud de piso desde cada nivel                                                       |
| 🔈 **Buzzer (PA8, PWM)**                            | Alarma sonora                               | Indica llegada de piso y apertura/cierre de puerta                                       |
| 💡 **LEDs indicadores (PB6–PB8)**                   | Un LED por piso                             | Muestran solicitudes activas                                                             |
| 🖥️ **LCD 16×2 (modo 4 bit)**                       | Interfaz de estado                          | Muestra estado actual (“Subiendo…”, “Esperando…”, “Cerrando puerta…”) y la cola de pisos |
| 🔢 **Display 7 segmentos (4 dígitos multiplexado)** | Indicador visual dinámico                   | Muestra piso actual y animación de flecha de movimiento                                  |
| ⚙️ **Motor Stepper 1 (PB12–PB15)**                  | Movimiento del elevador                     | Acciona una **polea** que eleva o baja la cabina                                         |
| ⚙️ **Motor Stepper 2 (PC8–PC11)**                   | Movimiento de puerta                        | Acciona un **engranaje grande** que abre o cierra la puerta                              |
| 🔌 **USART2 (PA2/PA3)**                             | Comunicación serie (115200 bps)             | Envío de **log de eventos** a la consola                                                 |
| 🧩 **Estructura 3D + Caja de acrílico**             | Elementos físicos                           | Soporte mecánico, estabilidad y protección del circuito                                  |

---

## 🔄 Lógica de Funcionamiento

El proyecto opera **enteramente por interrupciones**, cumpliendo la consigna de **no usar procesos dentro del `while(1)` principal**.

| Módulo                   | Timer / Fuente    | Frecuencia                                                        
| ------------------------ | ----------------- | ------------------------------------------------------------------
| ⏱ **TIM21**              | 1 kHz             | Multiplexado de displays y lectura del keypad                     
| ⏱ **TIM22**              | 100 Hz            | Actualización de LCD, animación y FSM general del elevador        
| ⏱ **TIM2**               | 1 kHz             | Control de bobinas de motores y buzzer por software               
| ⚡ **EXTI0-1 / EXTI4-15** | Entradas externas | Interrupciones por push buttons (con debounce físico)             
| 💬 **USART2**            | 115200 bps        | Registro de eventos (“Piso 2 solicitado”, “Puerta abriendo”, etc.)

### 🧩 Flujo de ejecución

1. **Keypad o botón** solicita un piso → interrupción EXTI o lectura periódica.
2. **Cola de destinos (`next_floors[3]`)** almacena la secuencia de pisos pendientes.
3. **FSM del elevador** determina dirección y activa el **motor stepper**.
4. Al llegar al piso:

   * Se detiene el motor.
   * Se acciona **buzzer** y **puerta** (abrir → esperar → cerrar).
   * Se actualiza el LCD y el log por **USART2**.
5. Todo el proceso es autónomo y concurrente mediante **interrupciones temporizadas**.

---

## 💻 Arquitectura del Software

* **Sin delays.** Todos los tiempos (buzzer, pasos del motor, animaciones) se controlan mediante **timers e interrupciones periódicas**.
* **Modularidad:**

  * `isr_elevator_motor_movement()`
  * `isr_door_motor_movement()`
  * `isr_display_digits_time_multiplexing()`
  * `isr_keypad_read()`
  * `isr_buzzer_beep()`
  * `isr_lcd_update()`
* **Comunicación UART (USART2):** genera un **log continuo** que documenta cada transición y evento del sistema.

---

## 🔩 Aspectos Físicos de la Maqueta

* **Cabina del elevador** impresa en 3D con polea accionada por motor paso a paso.
* **Puerta lateral** con engranaje circular de apertura controlada.
* **Caja inferior de acrílico** con los circuitos (Nucleo, motores, drivers ULN2003, resistencias de debounce y buzzer).

---

## 📹 Demostración

* 🎥 **Video del funcionamiento completo:**
  [🔗 Ver en YouTube](https://youtu.be/TU_LINK_DEL_VIDEO)

* 💻 **Explicación del código y análisis de interrupciones:**
  [🔗 Video técnico / explicación del código](https://youtu.be/TU_LINK_DE_EXPLICACION)

---
