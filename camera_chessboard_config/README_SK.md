Do configu kamery docasne nastav:

```yaml
save_images: true
output_dir: "~/basler_trigger_output_cpp"
```

Spusti kameru a nafot cca 20 az 40 obrazkov sachovnice z roznych uhlov.
Sachovnica musi byt ostra, viditelna cela a v roznych castiach obrazu.

Potom vrat:

```yaml
save_images: false
```

## 3. Vytvor jeden kalibracny subor

Priklad pre sachovnicu 9x6 vnutornych rohov a stvorcek 25 mm:

```bash
cd ~/ROS2-Camera-Sync-dominik/camera_chessboard_config
bash scripts/calibrate_from_saved_images.sh '~/basler_trigger_output_cpp/*.png' 9 6 25
```

Vystup:

```bash
~/ROS2-Camera-Sync-dominik/config/basler_chessboard_ros.yaml
~/ROS2-Camera-Sync-dominik/config/basler_intrinsics_simple.yaml
```

Dolezite: 9x6 su vnutorne rohy, nie pocet ciernych/bielych stvorcekov.

## 4. Pouzi config v Basler sync YAML

Do svojho `basler_ext_trigger_cpp_node` configu pridaj tieto riadky:

```yaml
calibration_file: "/home/jetson/ROS2-Camera-Sync-dominik/config/basler_chessboard_ros.yaml"
camera_info_topic: "/basler/camera_info"
publish_camera_info: true
```

Hotovy priklad je v:

```bash
camera_chessboard_config/config/basler_ext_trigger_cpp_with_calib.yaml
```

## 5. Spustenie pri synchronizacii

Terminal 1 - kamera ako doteraz, len s configom, ktory obsahuje `calibration_file`:

```bash
cd ~/ROS2-Camera-Sync-dominik
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py
```

Terminal 2 - CameraInfo publisher z toho isteho configu:

```bash
cd ~/ROS2-Camera-Sync-dominik/camera_chessboard_config
bash scripts/run_camera_info_from_config.sh ~/ROS2-Camera-Sync-dominik/config/basler_ext_trigger_cpp_with_calib.yaml
```

Kontrola:

```bash
ros2 topic list | grep basler
ros2 topic echo /basler/camera_info --once
ros2 topic hz /basler/image_raw
ros2 topic hz /basler/camera_info
```

Ak `/basler/camera_info` ide rovnakou frekvenciou ako `/basler/image_raw`, kalibracny config sa pouziva spravne.

## Poznamka

Toto riesenie nemusi menit C++ Basler node. C++ node dalej publikuje obraz a sync timestampy. Python node cita ten isty YAML config a ku kazdemu obrazu publikuje spravne CameraInfo z jednorazovej kalibracie.
