# Pruebas con sensores reales

Guía de traspaso para pasar el sistema de detección y seguimiento de personas
del ARCO del simulador Gazebo a un robot real con sensores reales. El código se
migró a ROS 2 Humble y se verificó en simulación; este documento cubre lo que
cambia para hardware real. Consulta [README.md](README.md) para la arquitectura
completa.

## Resumen — qué cambia respecto a la simulación

El sistema está totalmente parametrizado, así que pasar a sensores reales es
**configuración, no código**. Cambian tres cosas:

| Simulación | Robot real |
|---|---|
| `use_sim_time:=true` | **quítalo** (por defecto es `false`; los sensores reales usan el reloj del sistema) |
| `scan_topic:=/scan_raw`, cámaras remapeadas a topics del sim | remapea a los **topics reales de tus drivers** |
| `frame_id:=odom` (el `map` del sim no era fiable) | `frame_id:=map` normalmente funciona con localización real (AMCL/SLAM) |

## Estado: qué está verificado y qué queda abierto

- ✅ **Detector láser + fusionador**: verificado de principio a fin en
  simulación (sigue correctamente a personas caminando, comparado con el
  ground truth).
- ⚠️ **Detector de cámara**: tanto `people_rgbd` (HOG) como `people_yolo`
  (YOLOv4-tiny) funcionan correctamente pero **no** detectaban de forma fiable
  los *mallados sintéticos* de humanos del simulador. Con una **cámara real** se
  espera que funcionen mucho mejor — **esto es lo principal a validar en
  hardware real.**
- ⚠️ **Rama UWB** (`localino`/`roloc`): funcional, validada solo con datos de
  rango sintéticos — nunca probada con anclas/etiquetas UWB reales.

## Requisitos previos

```bash
# ROS 2 Humble + dependencias
sudo apt install libgsl-dev libopencv-dev \
     ros-humble-cv-bridge ros-humble-image-transport \
     ros-humble-message-filters ros-humble-tf2-ros \
     ros-humble-tf2-geometry-msgs ros-humble-vision-opencv

# Compilar
cd <ros2_ws>
colcon build --packages-up-to \
  people_laser_detector people_laser_tracker \
  people_rgbd people_yolo range_msgs localino roloc arco_people_tracker
source install/setup.bash
```

## Ejecución con sensores reales

Primero identifica tus topics y frames reales:
```bash
ros2 topic list                 # encuentra tus topics de scan + cámara
ros2 run tf2_tools view_frames  # comprueba el árbol TF / nombres de frames
```

### 1. Detector láser
```bash
ros2 launch people_laser_tracker people_tracker.launch.py \
  scan_topic:=/scan \
  frame_id:=map \
  params_file:=/ruta/a/laser_tracker_params.yaml
```

### 2. Detector de cámara (YOLO — recomendado frente a HOG)
Remapea cada topic de imagen a los nombres de tu driver. Ejemplo para una
RealSense:
```bash
ros2 launch people_yolo people_yolo.launch.py \
  rgb_image_topic:=/camera/color/image_raw \
  depth_image_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=map \
  params_file:=/ruta/a/people_yolo_params.yaml
```
- El nodo acepta profundidad en `32FC1` (metros) y `16UC1` (milímetros).
- **Frame de cámara:** los drivers reales publican un frame óptico adecuado
  (p. ej. `camera_color_optical_frame`), así que normalmente puedes dejar
  `camera_optical_frame` vacío en el config — el nodo usa el frame de la
  cabecera de la imagen. (En simulación tuvimos que inyectar un frame óptico
  estático porque el sim etiquetaba las imágenes con `base_footprint`.)
- La profundidad y el RGB **deben estar registrados/alineados** (misma
  resolución y punto de vista).

### 3. Rama UWB (opcional)
```bash
ros2 launch localino localino.launch.py params_file:=/ruta/a/anclas.yaml
ros2 launch roloc   roloc.launch.py   params_file:=/ruta/a/etiquetas.yaml
```
Pon las **posiciones reales medidas de las anclas** en el `anchor_list` de
`localino` y los **IDs reales de las etiquetas** en las secciones por nodo de
`roloc`.

### 4. Fusionador
```bash
ros2 launch arco_people_tracker people_tracker.launch.py \
  frame_id:=map \
  params_file:=/ruta/a/people_tracker_params.yaml
```
`frame_id` debe coincidir con el frame en el que publican los detectores.

## Cómo verificar que funciona

1. **Detectores publicando** — cada detector activo debe publicar en
   `/people_detections` con su etiqueta:
   ```bash
   ros2 topic echo /people_detections | grep -A1 tagnames   # LASER / RGBD / UWB
   ```
2. **Salida fusionada** — pistas consolidadas en `/people`:
   ```bash
   ros2 topic echo /people
   ```
3. **Visual (RViz)** — Fixed Frame = tu `frame_id`; añade displays MarkerArray
   en `/people_detections/markers` y `/people/markers`. Los marcadores deben
   quedar sobre las personas reales y seguirlas.
4. **Comprobación de sentido** — una persona caminando debe mostrar una
   velocidad realista distinta de cero (~0.3–1.5 m/s) y una posición estable;
   los falsos positivos estáticos se quedan a ~0 m/s.

## Aspectos a vigilar

- **El árbol TF debe conectar** los frames de los sensores con `frame_id`. Si
  ves avisos `TF ... failed`, la transformada del frame del scan/cámara a
  `frame_id` no está disponible — revisa la localización y
  `ros2 run tf2_ros tf2_echo`.
- **Falsos positivos del láser**: la detección de piernas 2D se engancha a patas
  de sillas/mesas. Ajusta `min_beams` / `threshold` / `max_distance`; la rama de
  cámara ayuda a desambiguar.
- **Cobertura del láser**: un láser 2D no ve a personas ocluidas o quietas — es
  lo esperado; por eso el sistema es multisensor.
- **Rendimiento de la cámara**: YOLOv4-tiny corre a ~28 fps en CPU. Si va lento,
  baja `input_size` (320) o `rate`; para más precisión usa un modelo YOLO mayor
  o un backend GPU. **Aquí encaja el trabajo del alumno de TFG de "actualizar el
  detector"** — cambiar a un modelo más moderno que publique en
  `/people_detections` con la etiqueta `RGBD` no requiere ningún cambio en el
  fusionador.

## Parámetros clave (en `config/*.yaml`)

- Láser: `threshold`, `min_beams`, `max_distance`, `legs_distance`,
  `track_distance`, `track_timeout`
- YOLO: `confidence_threshold`, `nms_threshold`, `input_size`, `depth_min/max`,
  `rate`, `camera_optical_frame`
- UWB: `anchor_list` (localino), `tag` / `name` por nodo (roloc)
- Fusionador: `track_distance`, `track_timeout`

Repositorio: https://github.com/ppueyor/people_tracking_ros2
