# ROS2 Camera Sync – Basler + LiDAR

Tento workspace obsahuje ROS 2 balík pre Basler kameru spúšťanú externým triggerom a synchronizovanú s LiDAR timestampami. Projekt je určený ako kamerová časť systému pre FAST-LIVO2 / LiDAR-kamera spracovanie.

## Princíp fungovania

Kamera je nastavená v režime externého triggera. Po každom trigger pulze kamera vytvorí snímku a ROS 2 node ju publikuje ako `sensor_msgs/Image` na topic:

```bash
/basler/image_raw
```

Synchronizácia je riešená softvérovo cez timestampy. LiDAR bridge zapisuje posledné LiDAR timestampy do shared memory súboru:

```bash
/dev/shm/lidar_stamp.bin
```

Kamerový node pri každej prijatej snímke vyhľadá najbližší LiDAR timestamp podľa host času prijatia. Ak je rozdiel menší ako nastavený threshold, image header kamery sa nastaví podľa LiDAR ROS timestampu. Tým pádom FAST-LIVO2 vidí kamerový obraz a LiDAR dáta s kompatibilnými časmi v ROS headeroch.

Táto synchronizácia neupravuje samotné obrazové dáta ani LiDAR pointcloud. Iba priradí kamere najbližší LiDAR timestamp v `msg->header.stamp`.

## Hlavné súbory

```text
run_all.sh
src/basler_ext_trigger_cpp/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── camera_params.yaml
│   └── cyclonedds.xml
├── launch/
│   └── ext_trigger_camera.launch.py
├── include/basler_ext_trigger_cpp/
│   ├── basler_ext_trigger_node.hpp
│   ├── lidar_stamp_reader.hpp
│   └── liv_mmap.hpp
└── src/
    └── ext_trigger_node.cpp
```

Význam hlavných headerov:

- `basler_ext_trigger_node.hpp` – deklarácia hlavného kamerového ROS 2 nodu.
- `lidar_stamp_reader.hpp` – číta LiDAR timestampy z `/dev/shm/lidar_stamp.bin`.
- `liv_mmap.hpp` – voliteľný zápis kamerových metadát do mmap ring bufferu. V aktuálnom nastavení je vypnutý cez `enable_mmap: false`.

## Konfigurácia

Hlavné nastavenia sú v:

```bash
src/basler_ext_trigger_cpp/config/camera_params.yaml
```

Dôležité parametre:

```yaml
image_topic: "/basler/image_raw"
frame_id: "basler_camera"
trigger_source: "Line1"
trigger_activation: "RisingEdge"
trigger_mode: "On"
exposure_time_us: 300.0
lidar_mmap_path: "/dev/shm/lidar_stamp.bin"
use_lidar_stamp_for_ros_header: true
match_threshold_ns: 40000000
resync_trigger_dt_ns: 40000000
resync_trigger_count: 3
enable_mmap: false
compat_stamp_path: ""
```

Hodnota `match_threshold_ns: 40000000` znamená 40 ms. Ak je tam napríklad `40000`, znamená to iba 40 mikrosekúnd, čo je väčšinou príliš prísne.

## Build

```bash
cd ~/ROS2-Camera-Sync-dominik
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Ak build nevie nájsť Pylon SDK, skontroluj:

```bash
/opt/pylon/bin/pylon-config --version
```

## Spustenie iba kamery

```bash
cd ~/ROS2-Camera-Sync-dominik
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py
```

## Spustenie celého systému

Na spustenie LiDAR drivera, LiDAR stamp bridge, Basler kamery, voliteľne IMU a FAST-LIVO2 slúži:

```bash
./run_all.sh
```

Skript otvorí samostatné terminály, nastaví ROS 2 prostredie, CycloneDDS, LiDAR sieťové rozhranie a spustí potrebné nody.

Príklady:

```bash
RUN_IMU=0 RUN_FAST_LIVO=0 RUN_RVIZ=False ./run_all.sh
RUN_RVIZ=False ./run_all.sh
RUN_MONITOR=1 ./run_all.sh
```

## Čo sa pri spustení vytvára

Projekt už neukladá `Matching/`, `Matches.csv`, `capture_log.csv` ani obrázky na disk. Pri spustení sa vytvárajú iba debug logy terminálov:

```bash
~/terminalz/run_DATUM_CAS/
```

Tieto logy slúžia na kontrolu spustenia a ladenie.

Do `/dev/shm` sa používajú dočasné shared memory súbory:

```bash
/dev/shm/lidar_stamp.bin
/dev/shm/liv_sync_ring.bin    # iba ak enable_mmap=true
/dev/shm/liv_sync_stamp       # iba ak compat_stamp_path nie je prázdny
```

## Overenie

Po spustení skontroluj topicy:

```bash
ros2 topic list
ros2 topic hz /lidar_points
ros2 topic hz /basler/image_raw
```

Header kamery:

```bash
ros2 topic echo /basler/image_raw --field header
```

Ak synchronizácia funguje, v logu kamery sa objavuje `quality=MATCHED` a image header timestamp sa nastavuje podľa najbližšieho LiDAR timestampu.

## Poznámky k odovzdaniu

Do odovzdania nie je nutné dávať `.git/`, `build/`, `install/`, `log/`, staré debug výstupy ani vygenerované runtime logy. Dôležitý je najmä `run_all.sh`, balík `src/basler_ext_trigger_cpp/` a tento README súbor.
