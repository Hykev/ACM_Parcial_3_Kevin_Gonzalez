# 🧮 Serie 2 – Implementación de FSM con Multiplexación de Displays y Programación en FPGA (Basys 3)

## 🧠 Descripción General

Esta práctica implementa y complementa la **máquina de estados finitos (FSM)** desarrollada en los parciales anteriores, ahora ejecutada en **hardware real** utilizando la **FPGA Basys 3**.  
El sistema combina la **lógica secuencial** de una garita de parqueo con una **interfaz visual multiplexada** que muestra tanto la información del sistema como un **reloj en formato 24 h**, todo dentro del mismo hardware.

El proyecto se divide en **dos módulos funcionales** principales:

1. **Sistema de Garita (FSM)**  
   Implementa dos máquinas de estados:
   - **M1 – Máquina de Sellado (Moore):** controla el proceso de validación del ticket al insertar monedas.  
   - **M2 – Máquina de Talanquera (Mealy):** controla la apertura y cierre de la barrera según el ticket y el sensor del vehículo.

2. **Reloj Digital (Clock 24 h)**  
   Genera y actualiza la hora local en formato HH:MM, visualizada en los displays de 7 segmentos.  
   Está implementado mediante contadores síncronos que simulan segundos, minutos y horas, con una base de tiempo ajustable.

Ambas funciones se integran dentro de un módulo superior (`Top_basys3`) que permite alternar entre las dos vistas usando el **botón U** de la Basys3.  
Por defecto, el sistema inicia mostrando la **FSM** y, al presionar el botón, cambia a la **vista de reloj**.

---

## ⚙️ Entradas y Salidas (FPGA)

| Señal | Tipo | Descripción |
|-------|------|-------------|
| `clk` | Entrada | Reloj de 100 MHz de la Basys 3 |
| `btnC` | Entrada | Reset global del sistema |
| `btnU` | Entrada | Alterna entre la vista de **FSM** y **reloj** |
| `sw[0]` | Entrada | Ticket insertado (`iT`) |
| `sw[1]` | Entrada | Moneda insertada (`iM`) |
| `sw[2]` | Entrada | Sensor del vehículo (`C`) |
| `an[3:0]` | Salida | Control de los 4 displays de 7 segmentos (activos en bajo) |
| `seg[6:0]` | Salida | Segmentos de los displays (activos en bajo) |
| `dp` | Salida | Punto decimal (apagado) |

---

## 💡 Multiplexación de los Displays de 7 Segmentos

El sistema utiliza los cuatro displays integrados de la Basys3 con **multiplexación temporal** controlada por un contador rápido.  
Cada 1 ms se activa uno de los displays con el valor correspondiente, dando la apariencia de que todos permanecen encendidos al mismo tiempo.

### Vistas disponibles:

- **Vista FSM:**  
  - **Display 3 (izq.)** → Estado de la talanquera (`TAL2`)  
  - **Display 0 (der.)** → Contador de monedas (`D1`)  
  - Los demás displays permanecen apagados.

- **Vista Reloj (Clock):**  
  - **Displays 3–2** → Horas (`HH`)  
  - **Displays 1–0** → Minutos (`MM`)

El cambio entre ambas vistas se realiza mediante el **botón U**, el cual pasa por un **módulo de debounce** y un **generador de pulso único** (`one_pulse`) para evitar rebotes y duplicaciones de señales.

---

## 🔩 Arquitectura del Sistema

### Módulos Principales:

- **`M1_Moore`**  
  FSM de tipo Moore para el conteo de monedas y validación del ticket.

- **`M2_Mealy`**  
  FSM de tipo Mealy para el control de la barrera según el ticket validado y el sensor de vehículo.

- **`Top_FSM`**  
  Integra las dos máquinas anteriores y genera las señales principales del sistema.

- **`clock`**  
  Implementa el reloj 24 h con contadores internos para segundos, minutos y horas. Incluye una base de tiempo configurable para acelerar las pruebas.

- **`display_7segments`**  
  Se encarga de la multiplexación y decodificación de los números BCD hacia los 4 displays de la Basys3.

- **`debouncer`** y **`one_pulse`**  
  Acondicionan la señal del botón U para generar una sola transición limpia por pulsación.

- **`clk_psc`**  
  Divisor de reloj para ralentizar las transiciones de la FSM y hacer visible su funcionamiento.

- **`Top_basys3`**  
  Módulo superior que conecta todos los anteriores, define la lógica de multiplexación entre reloj y FSM, y vincula los puertos físicos de la FPGA.

---

## 🎥 Evidencias en Video

### Ejercicio 1 – Implementación de la FSM

- **Demostración de FSM sin reloj (antes de la actualización del parcial):**  
  [https://youtu.be/XsA-RSCqKbk](https://youtu.be/XsA-RSCqKbk)  

- **Demostración de FSM con reloj (versión final):**  
  [https://youtu.be/OaL7UdaPSfI](https://youtu.be/OaL7UdaPSfI)  

- **Explicación del código:**  
  [https://youtu.be/86AQyynaU6M](https://youtu.be/86AQyynaU6M)

---

### Ejercicio 2 – Etapas del flujo de Vivado

Este ejercicio corresponde al **proceso de descarga del binario a la FPGA**, abarcando las cuatro etapas principales del flujo de diseño en Vivado:

1. **RTL Schematic (Elaborated Design)** – Representación lógica del sistema antes de la síntesis.  
2. **Synthesis Design** – Traducción del RTL a celdas físicas (LUTs, DSPs, flip-flops).  
3. **Implementation Design** – Colocación y ruteo físico en la FPGA, verificación de DRC y Slack.  
4. **Bitstream / Hardware Manager** – Generación del archivo .bit y programación en la Basys3.  

📺 **Explicación completa del flujo y las vistas gráficas:**  
[https://youtu.be/46htZkBVLcM](https://youtu.be/46htZkBVLcM)
