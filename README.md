# ROS2 Camera Sync - Basler + LiDAR

Tento workspace obsahuje ROS 2 balík `basler_ext_trigger_cpp` pre Basler kameru
spúšťanú externým triggerom a synchronizovanú s LiDAR timestampami. Projekt je
kamerová časť zostavy Basler + Hesai LiDAR + FAST-LIVO2.

Aktuálny checkout je samostatný kamerový workspace. Skript `run_all.sh` navyše
očakáva vedľajšie workspace:

```bash
~/ROS2-LIDAR-CAMERA-BRIDGE-main
~/ROS2-Camera-Sync-dominik
~/ROS2-FAST-LIVO2-WS
```

Ak máš tento repo inde, buď ho presuň/symlinkni na `~/ROS2-Camera-Sync-dominik`,
alebo uprav premennú `CAMERA_WS` v `run_all.sh`.

## Princíp fungovania

Kamera beží v režime externého triggera. Po každom trigger pulze vytvorí snímku
a ROS 2 node ju publikuje ako `sensor_msgs/Image` na topic:

```bash
/basler/image_raw
```

Synchronizácia prebieha softvérovo cez timestampy. LiDAR stamp bridge zapisuje
posledné LiDAR timestampy do shared memory súboru:

```bash
/dev/shm/lidar_stamp.bin
```

Kamerový node pri prijatej snímke vyberie najbližší LiDAR timestamp podľa host
času prijatia. Ak je rozdiel v povolenom thresholde, `msg->header.stamp` kamery
sa nastaví podľa LiDAR ROS timestampu. FAST-LIVO2 potom vidí kamerové a LiDAR
dáta s kompatibilnými časmi v ROS headeroch.

Synchronizácia nemení obrazové dáta ani LiDAR pointcloud. Mení iba timestamp v
ROS image headeri.

## Hlavné súbory

```text
run_all.sh
scripts/
├── install_dds_tuning.sh
├── launch_with_network_check.sh
└── source_cyclone_env.sh
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
│   └── lidar_stamp_reader.hpp
└── src/
    └── ext_trigger_node.cpp
```

Poznámky:

- `basler_ext_trigger_node.hpp` a `ext_trigger_node.cpp` obsahujú hlavný kamerový
  ROS 2 node.
- `lidar_stamp_reader.hpp` číta LiDAR timestampy z `/dev/shm/lidar_stamp.bin`.
- `launch/ext_trigger_camera.launch.py` nastavuje CycloneDDS aj Pylon runtime pre
  kamerový node.
- `scripts/install_dds_tuning.sh` slúži na trvalé nastavenie DDS/sysctl buffrov
  na Jetson-e.
- V tomto checkout-e nie je súbor `run_all_reconnect.sh`; hlavný top-level
  spúšťací skript je `run_all.sh`.

## Závislosti

Potrebné minimum pre kamerový workspace:

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-cv-bridge
```

Ďalej musí byť nainštalovaný Basler Pylon SDK tak, aby existovalo:

```bash
/opt/pylon/bin/pylon-config
```

Pre `run_all.sh` sú potrebné aj externé workspace pre LiDAR bridge a FAST-LIVO2,
funkčný terminál (`gnome-terminal` alebo `x-terminal-emulator`) a príkazy
`ros2`, `ping`, `ip`, `sysctl`.

## Build kamery

Z koreňa tohto workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Ak build nevie nájsť Pylon SDK, over:

```bash
/opt/pylon/bin/pylon-config --version
```

## Konfigurácia kamery

Hlavný konfiguračný súbor:

```bash
src/basler_ext_trigger_cpp/config/camera_params.yaml
```

Aktuálne dôležité parametre:

```yaml
image_topic: "/basler/image_raw"
frame_id: "basler_camera"
trigger_source: "Line1"
trigger_activation: "RisingEdge"
trigger_mode: "On"
exposure_time_us: 300.0
lidar_mmap_path: "/dev/shm/lidar_stamp.bin"
use_lidar_stamp_for_ros_header: true
match_threshold_ns: 80000000
resync_trigger_dt_ns: 80000000
resync_trigger_count: 3
load_user_set: "UserSet2"
enforce_monotonic_lidar_seq: true
```

`match_threshold_ns: 80000000` znamená 80 ms. Hodnota `40000` je iba 40 us a je
pre túto synchronizáciu väčšinou príliš prísna.

## Spustenie iba kamery

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py
```

Alternatívne môžeš pred launchom ručne nastaviť CycloneDDS/Pylon prostredie:

```bash
source scripts/source_cyclone_env.sh
ros2 launch basler_ext_trigger_cpp ext_trigger_camera.launch.py
```

Samotná kamera očakáva, že LiDAR bridge už zapisuje `/dev/shm/lidar_stamp.bin`.
Ak súbor ešte neexistuje, node beží ďalej a skúša reader znovu pripojiť.

## Spustenie celého systému

Na spustenie LiDAR drivera, LiDAR stamp bridge, Basler kamery, voliteľne IMU a
FAST-LIVO2 slúži:

```bash
./run_all.sh
```

Skript:

- nastaví kernel buffre pre ROS 2/CycloneDDS, ak sú nízke,
- nastaví LiDAR sieťové rozhranie,
- otestuje ping na LiDAR,
- vyčistí staré `/dev/shm` sync súbory,
- otvorí samostatné terminály pre jednotlivé procesy,
- uloží logy do `~/terminalz/run_DATUM_CAS/`.

Užitočné premenné:

```bash
LIDAR_IFACE=eno1
LIDAR_HOST_IP=192.168.1.100/24
LIDAR_SENSOR_IP=192.168.1.201
RUN_IMU=1
RUN_FAST_LIVO=1
RUN_RVIZ=True
RUN_MONITOR=0
RUN_REQUIRE_IMU_FOR_LIVO=1
KILL_OLD=1
```

Príklady:

```bash
RUN_IMU=0 RUN_FAST_LIVO=0 RUN_RVIZ=False ./run_all.sh
RUN_RVIZ=False ./run_all.sh
RUN_MONITOR=1 ./run_all.sh
LIDAR_IFACE=enp12s0 ./run_all.sh
RUN_REQUIRE_IMU_FOR_LIVO=0 ./run_all.sh
```

## Network-only helper

Skript `scripts/launch_with_network_check.sh` nastaví interface `eno1`, overí
ping na LiDAR `192.168.1.201` a spustí iba kamerový launch:

```bash
scripts/launch_with_network_check.sh
```

Ak používaš iný interface alebo IP adresy, uprav premenné priamo v tomto skripte
alebo použi flexibilnejší `run_all.sh`.

## Runtime súbory

Pri spustení vznikajú debug logy terminálov:

```bash
~/terminalz/run_DATUM_CAS/
```

V `/dev/shm` sa používajú dočasné shared memory súbory:

```bash
/dev/shm/lidar_stamp.bin
/dev/shm/liv_sync_ring.bin
/dev/shm/liv_sync_stamp
```

`run_all.sh` tieto tri súbory pred štartom vymaže, aby systém začínal z čistého
stavu. Aktuálny kamerový node priamo číta hlavne `/dev/shm/lidar_stamp.bin`.

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

Ak synchronizácia funguje, v logu kamery sa objavuje `quality=MATCHED` a image
header timestamp sa nastavuje podľa najbližšieho LiDAR timestampu.

## Čo neodovzdávať

Do odovzdania nepatrí `.git/`, `build/`, `install/`, `log/`, staré debug výstupy
ani vygenerované runtime logy. Dôležité sú najmä:

```text
README.md
run_all.sh
scripts/
src/basler_ext_trigger_cpp/
```
