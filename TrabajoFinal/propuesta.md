# ROS2LoopDetector

Se propone crear un nodo de [ROS2](https://docs.ros.org) que permita un comportamiento "plug & play" para la detección de ciclos basado en distintos tipos de información, con un comportamiento reconfigurable para cada subconjunto de tipos de información adicional que se le provee.

## Objetivos

Los objetivos esperados del nodo serán:
- ofrecer una interfaz de entrada modular,
- permitir la reconfiguración del sistema de detección según las entradas provistas,
- permitir configurar al sistema con distintos métodos en forma modular (hasta donde sea posible),
- ofrecer una interfaz de salida estándar, lo menos dependiente posible de las entradas,

y opcionalmente se intentará lograr:
- reconfiguración on-the-fly según las entradas provistas,
- ofrecer métodos que permitan múltiples entradas de información desfasada en el tiempo,
- ofrecer métodos que permitan la falta de elementos de entrada en distintos instantes de tiempo.

## Consideraciones

...