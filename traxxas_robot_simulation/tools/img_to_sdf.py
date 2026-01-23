#!/usr/bin/env python3
"""
image_to_sdf_ground.py
Convierte una imagen PNG en un plano de suelo texturizado para Gazebo (gz-sim).
El suelo tendrá la imagen PNG aplicada como textura, sin generar muros.

Edita CONFIG abajo, luego ejecuta: python3 src/mobile_robot/tools/image_to_sdf_ground.py
"""

import os

try:
    import cv2
except Exception:
    cv2 = None
try:
    from PIL import Image
except Exception:
    Image = None

try:
    import yaml
except Exception:
    yaml = None

# Obtener la ruta del paquete automáticamente desde la ubicación del script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)  # Sube un nivel desde tools/

# ----------------------- CONFIG (EDITA AQUÍ) -----------------------
CONFIG = {
    # Ruta al YAML del mapa de ROS (opcional)
    "MAP_YAML": "",  # e.g. "/home/you/maps/office.yaml" (deja "" para desactivar)

    # Si no hay MAP_YAML, usa estos valores:
    "IMAGE_PATH": os.path.join(PACKAGE_DIR, "tools", "track_img.png"),
    "RESOLUTION": 12/1363,     # metros/pixel
    "ORIGIN_X": None,        # coordenada X del mundo para el pixel (0,0)
    "ORIGIN_Y": None,        # coordenada Y del mundo para el pixel (0,0)
    "FLIP_Y": True,         # True: +Y hacia arriba en el mundo

    # Altura del plano (generalmente 0 para el suelo)
    "GROUND_HEIGHT": 0.0,   # metros

    # Salida
    "OUTPUT_SDF": os.path.join(PACKAGE_DIR, "worlds", "track_world.sdf"),
}
# ---------------------------------------------------------------

SDF_WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="floorplan_world">
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.2 0.2 0.2 1</background>
    </scene>
    
    <!-- Plano de suelo con textura de la imagen -->
    <model name="floorplan_ground">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{width} {height}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{width} {height}</size>
            </plane>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <ambient>1 1 1 1</ambient>
            <specular>0 0 0 1</specular>
            <emissive>1 1 1 1</emissive>
            <pbr>
              <metal>
                <albedo_map>{texture_path}</albedo_map>
                <metalness>0.0</metalness>
                <roughness>1.0</roughness>
                <emissive_map>{texture_path}</emissive_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
"""

def read_yaml_map(path):
    """Lee los parámetros de un archivo YAML de mapa de ROS."""
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    img_path = data["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(path), img_path)
    resolution = float(data["resolution"])
    origin = data.get("origin", [0, 0, 0])
    origin_x, origin_y = float(origin[0]), float(origin[1])
    return {
        "image": img_path,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
    }

def get_image_dimensions(path):
    """Obtiene las dimensiones de la imagen."""
    if cv2 is not None:
        img = cv2.imread(path)
        if img is None:
            raise RuntimeError(f"No se pudo leer la imagen: {path}")
        height, width = img.shape[:2]
        return width, height
    if Image is not None:
        img = Image.open(path)
        width, height = img.size
        return width, height
    raise RuntimeError("Se necesita OpenCV (python3-opencv) o Pillow (python3-pil) para leer imágenes.")

def main():
    cfg = CONFIG.copy()

    # Leer parámetros del YAML si está disponible
    if cfg["MAP_YAML"] and yaml is not None and os.path.exists(cfg["MAP_YAML"]):
        y = read_yaml_map(cfg["MAP_YAML"])
        cfg["IMAGE_PATH"] = y["image"]
        cfg["RESOLUTION"] = y["resolution"]
        cfg["ORIGIN_X"] = y["origin_x"]
        cfg["ORIGIN_Y"] = y["origin_y"]

    if not os.path.exists(cfg["IMAGE_PATH"]):
        raise RuntimeError(f"No se encontró la imagen: {cfg['IMAGE_PATH']}")

    # Obtener dimensiones de la imagen
    width_px, height_px = get_image_dimensions(cfg["IMAGE_PATH"])
    
    # Calcular dimensiones en metros
    res = float(cfg["RESOLUTION"])
    width_m = width_px * res
    height_m = height_px * res
    
    # Calcular ORIGIN para centrar la imagen en (0, 0)
    # Si ORIGIN_X/Y son None, los calculamos para que el centro quede en (0,0)
    if cfg["ORIGIN_X"] is None:
        cfg["ORIGIN_X"] = -width_m / 2.0
    if cfg["ORIGIN_Y"] is None:
        cfg["ORIGIN_Y"] = -height_m / 2.0
    
    # Calcular posición central del plano
    ox, oy = float(cfg["ORIGIN_X"]), float(cfg["ORIGIN_Y"])
    flip_y = bool(cfg["FLIP_Y"])
    
    # El centro del plano en coordenadas del mundo
    center_x = ox + (width_m / 2.0)
    center_y = oy + (height_m / 2.0 if flip_y else height_m / 2.0)
    center_z = float(cfg["GROUND_HEIGHT"])
    
    # Ruta absoluta a la textura
    texture_path = os.path.abspath(cfg["IMAGE_PATH"])
    
    # Crear archivo world
    world_content = SDF_WORLD_TEMPLATE.format(
        x=center_x,
        y=center_y,
        z=center_z,
        width=width_m,
        height=height_m,
        texture_path=texture_path
    )
    
    output_dir = os.path.dirname(cfg["OUTPUT_SDF"])
    os.makedirs(output_dir, exist_ok=True)
    
    with open(cfg["OUTPUT_SDF"], "w") as f:
        f.write(world_content)
    
    print(f"\n[image_to_sdf_ground] Generación completada!")
    print(f"  Archivo world: {cfg['OUTPUT_SDF']}")
    print(f"  Imagen: {cfg['IMAGE_PATH']}")
    print(f"  Dimensiones (px): {width_px}x{height_px}")
    print(f"  Dimensiones (m): {width_m:.2f}x{height_m:.2f}")
    print(f"  Resolución: {res} m/px")
    print(f"  Posición central: ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})")
    print(f"\nPara usar en Gazebo:")
    print(f"  ign gazebo {cfg['OUTPUT_SDF']}")

if __name__ == "__main__":
    main()