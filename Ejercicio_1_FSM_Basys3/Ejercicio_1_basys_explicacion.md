# 🚦 Proyecto: Máquina de Estados Finitos – Sistema de Garita (Basys3)

## 🧠 Descripción General

Este ejercicio implementa un **sistema de control de garita de parqueo** utilizando **máquinas de estados finitos (FSM)** diseñadas en **SystemVerilog** y ejecutadas en una **FPGA Basys3**.

El sistema combina dos máquinas de estados:

1. **M1 – Máquina de Sellado de Ticket (Moore)**  
   Se encarga de validar el ticket del usuario al ingresar monedas.  
   - Requiere **2 monedas** para validar el ticket.  
   - Controla las señales de ticket (`T`), validación (`V`) y conteo (`D`).

2. **M2 – Máquina de Control de Talanquera (Mealy)**  
   Abre y cierra la talanquera una vez el ticket ha sido validado.  
   - Depende del ticket validado y del sensor de vehículo.  
   - Controla los estados de la talanquera (`DOWN`, `UP_START`, `UP`, `DOWN_START`).

Ambas FSM se integran en un módulo superior (`Top`), que a su vez se conecta al módulo `Top_fpga` encargado de interactuar con la **Basys3** mediante **switches** y **LEDs**.

---

## ⚙️ Entradas y Salidas (FPGA)

| Señal | Tipo | Descripción |
|-------|------|--------------|
| `clk` | Entrada | Reloj físico de 100 MHz de la Basys3 |
| `btnC` | Entrada | Botón central usado como **reset** |
| `sw[0]` | Entrada | Ticket insertado (`iT`) |
| `sw[1]` | Entrada | Moneda insertada (`iM`) |
| `sw[2]` | Entrada | Sensor del carro (`C`) |
| `led[0–1]` | Salida | Estado de la **talanquera** (`TAL2`) |
| `led[2–3]` | Salida | Estado del **contador de monedas** (`D1`) |
| `led[4]` | Salida | Ticket detectado (`T1`) |
| `led[5]` | Salida | Ticket validado (`V1`) |
| `led[6]` | Salida | Reloj dividido (`clk_slow`) visible |


---

## 🖥️ Demostración en Video

🎬 **Video explicativo:**  

🔗 [Ver en YouTube](https://youtu.be/XsA-RSCqKbk)

