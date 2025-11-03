# Simulador Web de Manipulador 2R – Trazado de Trébol en Hoja

Este proyecto es un **simulador web interactivo** de un manipulador planar **2R** (dos grados de libertad, tipo brazo de dibujo) que traza un **trébol estilizado** sobre una hoja (A4 o Carta).

Está implementado en **React + TypeScript** y usa **Recharts** para mostrar los perfiles cinemáticos y dinámicos.

## Características principales

- Brazo 2R con eslabones de longitudes configurables (`l1`, `l2`).
- Hoja A4/Carta posicionada en el plano mediante (`dx`, `dy`).
- Trayectoria de trébol basada en la **superfórmula de Gielis**, ajustada:
  - Escala limitada (`esc ≤ 1.2`),
  - Rotación limitada (`rot ∈ [-45°, +45°]`),
  - Ajuste a la hoja con margen interno.
- **Inicio siempre a la izquierda**:
  - Calcula el punto más a la izquierda de la curva.
  - Añade un segmento desde un **HOME a la izquierda de la hoja** hasta ese punto.
- Remuestreo de la trayectoria para lograr **velocidad de punta constante** `v_tip`.
- Cálculo de:
  - Cinemática directa e inversa (FK/IK).
  - Perfiles de **ángulos, velocidades y aceleraciones articulares**.
  - **Jacobiano**, manipulabilidad y número de condición.
  - **Pares articulares** (gravedad + inercia simplificada).
- Visualización en tiempo real:
  - Hoja + margen.
  - Área de trabajo del 2R.
  - Cuadrado circunscrito del trébol.
  - Trayectoria guía y **rastro** del movimiento.
  - Mecanismo (eslabones + triángulo de unión base–codo–efector).
  - Retícula y sistema de coordenadas \[m\].
- Gráficas de:
  - Ángulos articulares \(\theta_1,\theta_2\).
  - Velocidades \(\omega_1,\omega_2\).
  - Aceleraciones \(\alpha_1,\alpha_2\).
  - Manipulabilidad μ y número de condición κ.
  - Pares articulares \(\tau_1,\tau_2\).
- Mini **“unit tests”** integrados para verificar partes clave del modelo.

---

## Requisitos

- Node.js (v16+ recomendado).
- Gestor de paquetes: `npm` o `yarn`.
- Proyecto React (TypeScript recomendado).

Dependencias principales:
- `react`
- `react-dom`
- `recharts`

