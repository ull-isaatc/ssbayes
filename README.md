<img src="icon.png" align="right" />

# Mobile Robots Localization Simulator V1.0
> Trabajo realizado por Iván Rodríguez

## Funcionamiento del entorno experimental

En el directorio raíz tenemos los dos entornos experimentales desarrollados para este trabajo, un entorno bidimensional utilizando MATLAB y un entorno tridimensional utilizando V-REP.
Estos entornos se encuentran en los directorios "Experimentos en Matlab" y "Experimentos V-REP" respectivamente.
Estos scripts siguen el algoritmo especificado en el capítulo 5 de la memoria.
Es importante inicializar todos los parámetros de forma correcta para que la estimación realizada por los filtros sea correcta, además si la simulación se realiza en V-REP hay que tener especial precaución con colocar la IP del equipo cliente de forma correcta.
Para lanzar V-REP y poder trabajar por medio de la API con MATLAB, es necesario lanzar el programa con la siguiente sentencia:

```sh vrep.sh -gREMOTEAPISERVERSERVICE_19999_false_true
```

Si queremos lanzar el programa sin ejecutar el modo gráfico la sentencia sería:

```sh vrep.sh -h -gREMOTEAPISERVERSERVICE_19999_false_true
```

Por otra parte, hemos añadido las escenas y los modelos utilizados en las simulaciones para que los experimentos realizados sean reproducibles.

## Herramientas necesarias

- [Matlab](https://www.mathworks.com/products/matlab.html) -  Herramienta de software matemático que ofrece un entorno de desarrollo integrado (IDE) con un lenguaje de programación propio.
- [V-REP](http://www.coppeliarobotics.com/) - Entorno interactivo de simulación de robots.
- [V-REP API](http://www.coppeliarobotics.com/helpFiles/en/apiOverview.htm) - Archivos necesarios para el correcto funcionamiento de la API de V-REP.

## Artículos y documentos de referencia
- ["Implementación y evaluación de un entorno experimental para localización de robots móviles usando filtros de Kalman."](https://riull.ull.es/xmlui/handle/915/2606) - *Iván Rodríguez*
