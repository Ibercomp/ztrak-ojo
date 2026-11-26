# ZTrak Ojo

Programa que hace que un ojo animatrónico impreso en 3D siga con la mirada a las personas que tiene delante. Lo ha desarrollado [Ibercomp](https://ibercomp.com) para su stand en MATELEC.

- **Artículo con la explicación completa:** [Construye el ojo de MATELEC: visión artificial y código abierto](https://ibercomp.com/ojo-matelec-codigo-abierto/) (blog de Ibercomp).
- **Diseño mecánico, piezas imprimibles y firmware:** [ibercomp/ojo-3d](https://github.com/ibercomp/ojo-3d).

## Inspiración

Inspirado en el ojo animatrónico del vídeo original de Jonny Poole, con un diseño propio de Ibercomp.

## Cómo funciona

1. Una webcam envía imágenes al ordenador mediante FFmpeg.
2. Tres modelos analizan cada imagen:
   - **YOLOX** detecta a las personas.
   - **RTMPose** estima los puntos de su cuerpo y su cabeza.
   - **YuNet** confirma la cara.
   La inferencia se hace en el propio ordenador con ONNX Runtime.
3. El programa elige a una persona y calcula hacia dónde debe mirar el ojo: a la cara si se ve con fiabilidad y, si no, al centro del cuerpo.
4. Las órdenes de movimiento llegan por USB al Arduino Nano, que genera los pulsos de los tres servos: giro, cabeceo y párpados.

El programa no identifica a nadie: sigue posiciones en la imagen. Las imágenes se procesan en memoria y no se graban ni se envían por red.

Está escrito en Rust 2024, con interfaz nativa [egui/eframe](https://docs.rs/eframe).

## Requisitos

- Rust (edición 2024) y las herramientas de compilación del sistema.
- FFmpeg en el PATH o junto al ejecutable.
- Una webcam. Puede ir fija junto al ojo o dentro de la pupila.
- El mecanismo de [ojo-3d](https://github.com/ibercomp/ojo-3d) con su firmware:
  - **Recomendado:** Arduino Nano con [`UNO_AVR`](https://github.com/ibercomp/ojo-3d/tree/main/ViVi_DOIT_3_servos/UNO_AVR), por USB.
  - **Alternativa:** placa ESP8266 con [`ESP8266_RTOS`](https://github.com/ibercomp/ojo-3d/tree/main/ViVi_DOIT_3_servos/ESP8266_RTOS), por Wi-Fi o UART.

Los modelos ONNX están en [`models/`](models/README.md).

## Compilar y arrancar

### Linux

```sh
cargo build --release --locked
./target/release/ztrak-ojo
```

`iniciar.sh` compila si hace falta y arranca el programa. La configuración se guarda en `~/.config/ztrak-ojo/config.json`. Para abrir el puerto serie, tu usuario debe pertenecer al grupo `dialout`.

### Windows

El paquete para Windows se prepara desde Linux:

```sh
tools/compilar_windows.sh
```

Genera `dist/ztrak-ojo-windows/` y `dist/ztrak-ojo-windows.zip`. Incluyen:
- `ZTrak Ojo.cmd` y la carpeta `programa/`;
- FFmpeg, ONNX Runtime con DirectML y las bibliotecas de ejecución.

También se puede compilar en Windows con `cargo build --release --locked` y la toolchain MSVC. La configuración se guarda en `%APPDATA%\ztrak-ojo\config.json` y el registro en `ztrak-ojo-log.txt`, junto a `ZTrak Ojo.cmd`.

Lo hemos probado en un i3-12100F con una Intel Arc A310. El esqueleto corre en la gráfica con DirectML; el detector y las caras van en CPU y la interfaz usa DirectX 12. Las salidas inválidas de la IA (ceros o NaN) se descartan y nunca llegan al seguimiento.

En Windows el programa nunca elige webcam por su cuenta: la eliges en el selector de arriba y se guarda. Si en el mismo PC está instalada la aplicación ZTrak de Ibercomp (el oso del stand), su webcam aparece bloqueada para que el ojo no la use. Si hay dos webcams iguales, se distinguen por su identificador de Windows.

## Uso

1. **Cámara:** elige la webcam. Si el sensor está montado boca abajo, activa el giro de 180°.
2. **Ojo:** elige la conexión.
   - **UART / USB-serie** para el Nano, a 250000 baudios.
   - **Wi-Fi** para la placa ESP8266, indicando su IP.
3. **Montaje:** marca si la cámara va en la pupila, o indica dónde está respecto al ojo si va fija.
4. Pulsa **Aplicar y guardar ajustes**.
5. **Activar seguimiento** enciende los servos y sigue a la persona elegida. **Parar y apagar** detiene el control y apaga las salidas.

El programa arranca siempre sin mover el ojo. Con `--seguir`, activa el seguimiento al arrancar, para un stand sin teclado. Nunca lo activa si la webcam no está elegida, es ambigua o es la de ZTrak.

La pestaña **Manual** permite mover el ojo con el ratón y editar la curva de límites combinados de giro y cabeceo. La curva se guarda en la placa.

**Vista espejo** solo cambia cómo se muestra la imagen. El sentido de giro de los servos se corrige con las inversiones de **Montaje**.

## Montajes

### Cámara en la pupila

La cámara se mueve con el ojo. Un PID (`src/pid.rs`) convierte el error de la cara respecto al centro de la imagen en velocidad de mirada, con zona quieta, límite de velocidad y anti-windup. No necesita conocer la distancia.

`pupil_scale` convierte píxeles en grados de orden y `backlash_deg` compensa la holgura del mecanismo. `config/windows-inicial.json` trae los valores que medimos en nuestro montaje.

### Cámara externa

Indica la posición de la cámara respecto al ojo, en centímetros, vista desde detrás del ojo: X a la derecha, Y arriba y Z hacia delante.

El programa reconstruye la posición de la persona con el rayo de la cámara y una distancia. La distancia puede ser fija o estimarse por el ancho de hombros. Con una sola cámara el resultado es aproximado, y lo es más a corta distancia.

## Comportamiento

- **Foco:** mantiene a la misma persona mientras sea visible, con margen ante oclusiones breves. Cuando hay varias personas quietas, reparte la atención entre ellas cada 2–4 s. Quien se mueve toma la atención.
- **Cara o cuerpo:** mira a la cara cuando YuNet la confirma. Si la cara deja de verse pero la persona sigue, mira al centro de su cuerpo.
- **Parpadeo y gestos:** el firmware parpadea cada pocos segundos. Si alguien mira de frente y quieto, el ojo lo «observa con curiosidad» entrecerrando los párpados.
- **Búsqueda, aburrimiento y sueño:**
  - Si pierde a la persona, busca a los lados.
  - Si la persona se queda quieta mucho tiempo, se aburre y entrecierra.
  - Sin nadie delante, se duerme.
  - Despierta al detectar movimiento en la imagen.
  - Los tiempos se ajustan en la interfaz.
- **Seguridad:**
  - Las consignas respetan la envolvente mecánica que devuelve la placa.
  - Si dejan de llegar órdenes, el firmware frena.
  - Al cerrar el programa, se apagan las salidas.

## Comunicación con el ojo

| Placa | Enlace | Protocolo |
|---|---|---|
| Arduino Nano / Uno (`UNO_AVR`) | USB-serie a 250000 baudios | Binario con COBS y CRC16: [`PROTOCOLO_UART.md`](https://github.com/ibercomp/ojo-3d/blob/main/ViVi_DOIT_3_servos/UNO_AVR/PROTOCOLO_UART.md) |
| ESP8266 (`ESP8266_RTOS`) | Wi-Fi | Sesión por HTTP y seguimiento por UDP en el puerto 4210 |
| ESP8266 (`ESP8266_RTOS`) | UART0 a 230400 baudios | Líneas `@OJO <n> <orden>` |

Las órdenes no se acumulan en cola: solo cuenta la más reciente. La posición que devuelve la placa es su consigna, no una medida del servo.

## Pruebas y diagnóstico

```sh
cargo test --locked
./target/release/ztrak-ojo --camaras                 # lista webcams
./target/release/ztrak-ojo --modos "video=Nombre"    # modos de una webcam
./target/release/ztrak-ojo --prueba-camara           # 15 s de cámara y modelos, sin mover el ojo
./target/release/ztrak-ojo --prueba-camara --archivo video.mp4
./target/release/ztrak-ojo --prueba-enlace           # 60 lecturas de estado, sin mover el ojo
```

Si defines `ZTRAK_OJO_TRACE=/ruta/nueva.jsonl`, se guarda un registro local de hasta 120 s con escenas, órdenes y estado. Por defecto no se graba nada.

En `examples/` hay pruebas de comportamiento contra el simulador del firmware (`tools/simular_motor.py` de `ESP8266_RTOS`). `verificar_comportamiento -- --mover-ojo` es una prueba física voluntaria que mueve el ojo real.

## Licencia

El código está bajo licencia [MIT](LICENSE). Los modelos de `models/` son de terceros y mantienen su propia licencia (ver [`models/README.md`](models/README.md)).
