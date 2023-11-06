# MODIFICACIONES AL SCRIPT `localization.py`
El script `localization.py` se modificó para permitir las corridas múltiples automáticas, variando los factores correspondientes. 

El script se puede ejecutar de la misma manera que antes pero se le agregaron algunas banderas extras:
```
    --num_runs <n>: número de corridas para cada caso de simulación. 
                    Corre 'n' veces la simulación con los mismos factores
    --loop-factors: si se pone esta bandera, corre la simulación variando
                    los factores según el vector r del informe.
    --no-dataloop: se usa combinada con "--loop-factors". 
                    Desactiva la variación de los data-factors
    --loop-particles: se usa en el filtro de partículas. Ejecuta la simulación
                    variando la cantidad de partículas según el enunciado.
```

## Ejemplos de uso
Por ejemplo, para correr cada ítem del TP se debe ejecutar de la siguiente manera:

3.a)
```
    localization.py ekf --plot
```
3.b)
```
    localization.py ekf --loop-factors
```
3.c)
```
    localization.py ekf --loop-factors --no-dataloop
```

Análogo para el ítem 4.3 a, b y c.
4.d)
```
    localization.py pf --loop-factors --loop-particles
```

## Logueo de Resultados
Los resultados de cada simulación son guardados en los archivos `result-*.json`
Estos archivos tienen la siguiente codificación en su nombre:
`result-[filter]-<fl>-<ndl>.json` donde:
```
filter: nombre del filtro {ekf, pf}
fl: aparece si se corrió con la bandera --loop-factors
ndl: aparece si se corrió con la bandera --no-dataloop
pl: aparece si se corrió el PF con la bandera --loop-particles

```

# Script de Post-proceso `postprocess.py`
Este script carga los resultados logueados, procesa los datos y los grafica o muestra datos, etc.
Uso:
```
postprocess.py --data <filename>
```
donde `<filename>` es el nombre del archivo de resultados. P. ej: `results-pf-fl.json`

Muestra 6 gráficas. Gráficas 1-3 siempre y 4-6 sólo si se corrió con `--loop-factors`:
1. Muestra el error MPE de cada corrida para cada 'r' utilizado
2. Idem 1. pero error MME
3. Idem 1. pero ANEES
4. Grafica error MPE medio (entre todas las corridas por cada 'r') versus 'r'
5. Idem 4. pero con MME
6. Idem 4. pero con ANEES

