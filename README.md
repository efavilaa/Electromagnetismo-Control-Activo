# Confinamiento Magnético Activo en un Toroide

Proyecto desarrollado para la materia **Modelación Computacional de Sistemas Electromagnéticos** (ITESM Campus Monterrey).

## Descripción

Este proyecto simula en MATLAB el confinamiento magnético activo de un protón dentro de un toroide formado por 16 cables con corrientes alternas. El sistema implementa un mecanismo de control que detecta cuándo la partícula se aproxima al límite de confinamiento y activa corrientes de rescate para redirigirla hacia la región segura.

La simulación incluye:

* Cálculo del campo magnético mediante la ley de Biot-Savart.
* Aplicación de la fuerza de Lorentz para modelar el movimiento de la partícula.
* Sistema de control sectorizado con corrientes de rescate adaptativas.
* Comparación entre los métodos numéricos de Euler, Runge-Kutta de segundo orden (RK2) y Runge-Kutta de cuarto orden (RK4).
* Animación de la trayectoria y visualización de los cables activos durante el rescate.
* Análisis de conservación de energía y eficiencia del sistema de confinamiento.

## Resultados Principales

| Método | Tiempo en rescate |
| ------ | ----------------- |
| Euler  | 0.05%             |
| RK2    | 0.19%             |
| RK4    | 20.64%            |

Aunque RK4 activa con mayor frecuencia la corriente de rescate, fue el método que mejor conservó la energía cinética de la partícula, proporcionando la simulación físicamente más confiable.

## Integrantes

* Ana Elisa Celaya Montalvo (A01287120)
* Ethiel Favila Alvarado (A00844789)
* Catherine González Díaz (A00845539)
* Diego Ricardo Cango Clavijo (A00843937)

## Video de la Simulación

https://youtu.be/p_uOHhcvaKM
