# Basler external trigger ROS 2 workspace (C++)

Tento workspace používa **pylon C++ API** a je postavený v štýle oficiálnych Basler C++ samples:
- `PylonInitialize()` / `PylonTerminate()`
- `CInstantCamera`
- `StartGrabbing(...)`
- `CGrabResultPtr`
- `RetrieveResult(...)`

Node je navrhnutý pre režim:
- STM32 posiela externý trigger na **Line1**
- kamera vytvára frame na každý trigger
- node iba vyberá hotové grab results
- ukladá obrázky
- zapisuje CSV
- zapisuje metadata do mmap ring bufferu v `/dev/shm/liv_sync_ring.bin`
- zapisuje kompatibilný stamp do `/dev/shm/liv_sync_stamp`
- voliteľne vie publikovať `sensor_msgs/Image`

## Bezpečnostný princíp

Táto verzia **nič trvalo nezapisuje do UserSet1**.
Najprv testuj iba runtime konfiguráciu. To je úmyselne bezpečnejšie.

## Štruktúra

- `src/basler_ext_trigger_cpp/src/ext_trigger_node.cpp` — hlavný C++ node
- `src/basler_ext_trigger_cpp/include/basler_ext_trigger_cpp/liv_mmap.hpp` — mmap ring buffer
- `src/basler_ext_trigger_cpp/config/camera_params.yaml` — parametre
- `src/basler_ext_trigger_cpp/launch/ext_trigger_camera.launch.py` — launch

## Build

```bash
cd ~/basler_ext_trigger_cpp_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Ak `pylon-config` nie je v PATH, skontroluj:

```bash
which pylon-config
/opt/pylon/bin/pylon-config --version
```

## Spustenie

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py
```

## Očakávané správanie

- node otvorí kameru
- načíta `Default` do runtime stavu
- nastaví externý trigger cez `Line1`
- zapne `ChunkLineStatusAll`
- zavolá `StartGrabbing(...)`
- čaká na externé pulzy
- pri každom triggeri uloží 1 frame a metadata

## Výstupy

V `output_dir`:
- `img_000001.png`, ...
- `capture_log.csv`

V shared memory:
- `/dev/shm/liv_sync_ring.bin`
- `/dev/shm/liv_sync_stamp`

## Poznámky

- Predvolený formát je `png`, čo je na Linuxe bezpečná voľba.
- Ak neskôr budeš chcieť publish na ROS topic, nastav `publish_ros: true`.
- Ak neskôr budeš chcieť persistovať konfiguráciu do `UserSet1`, urob to až po úspešnom runtime teste.
