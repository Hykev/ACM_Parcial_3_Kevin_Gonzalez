# 🧩 Examen Parcial 3 – ACM (Arquitectura de Computadoras y Microcontroladores)

## 🧠 Introducción General

Este repositorio contiene el desarrollo completo del **Examen Parcial 3** de la asignatura **ACM – Arquitectura de Computadoras y Microcontroladores**, compuesto por **dos series prácticas** que integran el trabajo teórico y de laboratorio en hardware real.

Cada serie cuenta con su propia implementación, documentación y demostración en video.  
Los archivos `.md` incluidos en este repositorio explican a detalle el funcionamiento, arquitectura y resultados obtenidos en cada ejercicio.

---

## ⚙️ Serie 1 – Control de Elevador con STM32 Nucleo L053R8

Esta primera serie implementa un **sistema de control de elevador de tres pisos**, desarrollado completamente en lenguaje **C bare-metal** sobre la plataforma **STM32 Nucleo-64 L053R8**.  
El sistema integra:

- Máquina de estados finitos (FSM) para el control del movimiento, puerta y tiempos de espera.  
- Motores paso a paso para desplazamiento y apertura/cierre.  
- Teclado matricial y botones externos con interrupciones EXTI.  
- Visualización en displays 7 segmentos y LCD 16×2.  
- Comunicación serial (USART2) y buzzer de señalización.

---

## 💡 Serie 2 – Implementación de FSM en FPGA (Basys 3)

La segunda serie traslada el concepto de máquina de estados a una **implementación digital en FPGA Basys 3**, utilizando lenguaje **Verilog HDL**.  
El proyecto combina dos módulos principales:

1. **FSM de Garita de Parqueo:** Control de ticket, monedas y talanquera.  
2. **Reloj 24 h:** Visualizado en los displays 7 segmentos mediante multiplexación temporal.  

Incluye además la descripción del flujo completo de **Vivado**, desde el RTL schematic hasta la generación del bitstream final.

---

## 🎥 Evidencias

Cada serie incluye enlaces a videos de:
- **Demostración física del proyecto.**  
- **Explicación técnica del código y la implementación.**
