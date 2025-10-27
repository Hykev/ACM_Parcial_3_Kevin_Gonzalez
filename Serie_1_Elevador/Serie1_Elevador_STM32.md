# 🏗️ Serie 1 – Control de Elevador con Microcontrolador STM32 (Nucleo L053R8)

## 🧠 Descripción General

Este proyecto implementa un **sistema de control de elevador real** utilizando la **tarjeta STM32 Nucleo-64 (L053R8)**.  
El sistema integra una **máquina de estados finitos (FSM)** que coordina el movimiento del elevador entre tres pisos, la apertura/cierre de la puerta, la lectura de teclas y botones externos, así como la visualización de estados y cola de destinos mediante una **LCD 16×2** y **displays de 7 segmentos**.

El objetivo es demostrar el control secuencial y modular del sistema mediante **interrupciones, multiplexación y temporización precisa**, todo implementado a nivel de **registro (bare-metal)**.

---

## ⚙️ Componentes y Periféricos Utilizados

| Módulo | Descripción | Pines en STM32 |
|:--|:--|:--|
| **Displays 7 segmentos (4 dígitos)** | Visualizan el piso actual y la animación de movimiento (flecha ↑ / ↓) | PB0–PB5 (segmentos a,b,c,d,e,g) · PC0–PC3 (selectores de dígito) |
| **LCD 16×2 (modo 4 bits)** | Muestra estado actual y cola de destinos | PC4–PC7 (D4–D7), PB10 (E), PB11 (RS) |
| **Keypad 1×3 (interior)** | Selección de piso desde la cabina | PA0, PA1, PA4 (columnas) · PA7 (fila) |
| **Botones externos (llamado por piso)** | Solicitan el elevador desde cada nivel | PA6 (P1), PA9 (P2), PC12 (P3) |
| **Leds de piso** | Indican llamadas activas | PB6 (P1), PB7 (P2), PB8 (P3) |
| **Motor principal (elevador)** | Controla el movimiento entre pisos | PB12–PB15 (IN1–IN4) |
| **Motor secundario (puerta)** | Abre y cierra la puerta automáticamente | PC8–PC11 (IN1–IN4) |
| **Buzzer pasivo** | Señal audible al llegar a un piso | PA8 |
| **USART2** | Envío de mensajes de depuración por consola | PA2 (TX), PA3 (RX) |
| **LED de actividad** | Indicador de funcionamiento del temporizador principal | PA5 |

---

## 💡 Funcionalidad del Sistema

### 1. Movimiento del Elevador
- Se controla mediante un **motor paso a paso** (PB12–PB15) con dirección y pasos definidos por variables calibrables.  
- Cada transición entre pisos se realiza con una cantidad de pasos configurable (`steps_between[from][to]`).  
- La **FSM principal** controla los estados `IDLE`, `MOVING_UP`, `MOVING_DOWN` y `HOLD_AT_FLOOR`.  
- Al llegar a un piso, se ejecuta la **secuencia de puerta** (abrir → esperar → cerrar).

### 2. Puerta Automática
- Se acciona con un segundo motor paso a paso (PC8–PC11).  
- Posee tiempos y pasos calibrables:  
  - `door_steps_open` → cantidad de pasos al abrir.  
  - `door_steps_close` → cantidad de pasos al cerrar.  
  - `door_hold_ms_cfg` → tiempo con la puerta abierta.  

### 3. Teclado y Botones
- El **keypad interno (1×3)** permite seleccionar el destino desde la cabina.  
- Los **botones externos (EXTI)** solicitan el elevador desde cada piso.  
- Las solicitudes se agregan a una **cola circular** de máximo 3 niveles, sin duplicados.  
- Cada botón o tecla activa el **LED del piso** correspondiente mientras la petición está pendiente.

### 4. Visualización y Monitoreo
- Los **displays 7 segmentos** muestran el **piso actual** (primer dígito) y una **animación** (flecha ascendente o descendente) en los otros tres, mediante multiplexación controlada por `TIM22`.  
- La **LCD 16×2** muestra:  
  - **Línea 1:** estado del sistema (`Subiendo`, `Bajando`, `Esperando`, `Abriendo`, `Cerrando`).  
  - **Línea 2:** la cola actual, por ejemplo `Cola: [2 3]`.  

### 5. Señales y Sonido
- El **buzzer (PA8)** se activa durante la llegada a un piso o apertura de puerta.  
- Todos los eventos relevantes (movimiento, llegada, solicitudes, errores) se envían también al **USART2 @ 9600 bps**, visible desde la consola del IDE.

---

## 🔩 Arquitectura de Software

El programa se organiza en módulos de bajo nivel y rutinas de interrupción:

| Módulo | Función principal |
|:--|:--|
| **`GPIO_init_all()`** | Configura todos los pines usados. |
| **`TIM21` (1 ms)** | Temporizador de control general: FSM, keypad, animación, LCD refresh, buzzer timeout. |
| **`TIM22` (≈2 kHz)** | Multiplexación del display 7 segmentos. |
| **`TIM2` (10 kHz)** | PWM software para buzzer y control de motores stepper. |
| **`USART2_IRQHandler`** | Comunicación serial para depuración y comandos de prueba. |
| **`EXTI4_15_IRQHandler`** | Interrupciones externas de los botones de piso. |
| **`FSM principal`** | Controla transición entre pisos, estados de movimiento y puertas. |
| **`LCD_driver`** | Envío de comandos y datos en modo 4 bits, dividido en nibbles. |

---

## ⚙️ Parámetros de Calibración

Las distancias, velocidades y tiempos pueden ajustarse fácilmente desde el código principal (`main()`):

```c
// Distancia entre pisos (pasos de motor)
set_steps_between(1,2,2400);
set_steps_between(2,3,2400);

// Velocidad del motor del elevador
set_stepper_rate(400);  // pasos/seg

// Configuración de la puerta
door_steps_open  = 1000;
door_steps_close = 1000;
door_hold_ms_cfg = 1000;   // tiempo abierta en ms
```

Esto permite calibrar el sistema real según la altura entre pisos y la mecánica de la maqueta.

---

## 🧩 Máquina de Estados Principal

```text
               +----------+
               |   IDLE   |
               +----------+
                    |
          +---------+---------+
          |                   |
     [solicitud ↑]        [solicitud ↓]
          |                   |
   +-------------+     +---------------+
   | MOVING_UP   |     | MOVING_DOWN   |
   +-------------+     +---------------+
          |                   |
          +---------+---------+
                    |
                [llegada]
                    |
              +-------------+
              | HOLD_AT_FLOOR|
              +-------------+
                    |
             [abrir → esperar → cerrar]
                    |
                +----------+
                |   IDLE   |
                +----------+
```

---

## 🎥 Evidencias en Video

- **Demostración física del elevador:**  
  [https://youtu.be/XXXXXXXXXXX](https://youtu.be/XXXXXXXXXXX)

- **Explicación del código:**  
  [https://youtu.be/YYYYYYYYYYY](https://youtu.be/YYYYYYYYYYY)
