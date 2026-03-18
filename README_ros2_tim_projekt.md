# ROS2 TIM projekt – Basler / mock trigger / priprava na STM32 + Hesai

Tento README popisuje, ako rozbehat aktualny workspace `ros2_tim_projekt`, co presne robi, v akych rezimoch funguje a co je mozne otestovat aj bez STM32 a bez LiDARu.

---

## 1. Co je momentalne funkcne

Aktualne je overene, ze vo workspace funguje:

- **Basler kamera** cez `pylon_ros2_camera_wrapper`
- **mock trigger** v baliku `liv_sync`
- **pairing node**
- **dataset logger node**
- ukladanie obrazkov do `dataset/images`
- zapis zaznamov do `dataset/pairs.csv`

To znamena, ze uz teraz je mozne testovat pipeline:

**Basler -> trigger -> pairing -> dataset logger**

bez potreby STM32 a bez potreby Hesai LiDARu.

---

## 2. Co zatial NIE je otestovane

Bez dalsieho hardveru zatial nie je mozne overit:

- realne trigger pulzy zo STM32 cez seriovy port
- realne cloud data z Hesai LiDARu
- finalne parovanie **kamera + trigger + cloud**

Ak je pri sebe iba kamera, je normalne, ze:

- v `pairs.csv` je vyplneny `image_file`
- `cloud_file` ostava prazdny
- `dt_cloud_us` ostava prazdny

---

## 3. Struktura projektu

Priblizne dolezite casti projektu:

- `run_basler.sh` – spusti samotnu Basler kameru
- `run_sync_stack.sh` – spusti cely test stack
- `config/basler_preview_color.yaml` – konfiguracia Basler kamery pre farebny preview profil
- `config/basler_slam_mono.yaml` – konfiguracia Basler kamery pre mono profil
- `src/liv_sync/` – vlastny synchronizacny balik
  - `mock_trigger_node.py`
  - `stm32_trigger_node.py`
  - `pairing_node.py`
  - `dataset_logger_node.py`
- `dataset/`
  - `images/` – ukladane obrazky
  - `clouds/` – sem sa budu ukladat cloudy, ked bude pripojeny LiDAR
  - `pairs.csv` – zapisane sparovane zaznamy

---

## 4. Build workspace

Vzdy v novom terminali:

```bash
cd ~/ros2_tim_projekt
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
```

Ak su v `src/` stare alebo nepouzivane Livox baliky, nechaj ich ignorovane:

```bash
echo "Temporarily disabled for Basler+Hesai Humble workspace" > src/LIV_handhold/COLCON_IGNORE
```

Instalacia zavislosti a build:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
chmod +x run_basler.sh
chmod +x run_sync_stack.sh
chmod +x src/liv_sync/liv_sync/*.py
```

---

## 5. Spustenie iba kamery

Na test, ci kamera sama funguje:

```bash
cd ~/ros2_tim_projekt
source /opt/ros/humble/setup.bash
source install/setup.bash
./run_basler.sh color
```

Ak chces mono profil:

```bash
./run_basler.sh slam
```

Co je pri spusteni normalne:

- kamera sa najde a zacne publikovat `image_raw`
- moze sa zobrazit upozornenie, ze kamera nie je kalibrovana
- moze sa zobrazit upozornenie na prazdny `camera_info_url`

To nie je fatalna chyba pre obycajny preview a ukladanie obrazkov.

---

## 6. Spustenie test stacku bez STM32 a bez LiDARu

Toto je odporucany rezim, ked je k dispozicii iba kamera:

```bash
cd ~/ros2_tim_projekt
source /opt/ros/humble/setup.bash
source install/setup.bash
./run_sync_stack.sh mock color
```

Argumenty:

- prvy argument: `mock` alebo `stm32`
- druhy argument: `color` alebo `slam`

Priklady:

```bash
./run_sync_stack.sh mock color
./run_sync_stack.sh mock slam
```

V tomto rezime sa spusti:

- Basler kamera
- `mock_trigger_node.py`
- `pairing_node.py`
- `dataset_logger_node.py`

A uklada sa:

- obrazok do `dataset/images`
- zaznam do `dataset/pairs.csv`

Bez LiDARu je normalne, ze `cloud_file` ostane prazdny.

---

## 7. Spustenie s STM32

Ked bude pripojene STM32, pouzi:

```bash
./run_sync_stack.sh stm32 color
```

Aktualne sa predpoklada seriovy port:

- `/dev/ttyUSB0`
- baudrate `115200`

Ak bude STM32 na inom porte, treba upravit `run_sync_stack.sh`.

---

## 8. Spustenie s Hesai LiDARom neskor

V `run_sync_stack.sh` je pripravena cast pre Hesai, ale moze byt zatial zakomentovana. Ked bude realny LiDAR k dispozicii, treba odkomentovat launch Hesai drivera a skontrolovat:

- nazov cloud topicu
- typ spravy
- ci `pairing_node.py` pocuva spravne na cloud topic

Ocakavany ciel pre plny stack:

**Basler + trigger + Hesai -> sparovanie -> ulozenie image + cloud + CSV zaznam**

---

## 9. Kontrolne prikazy

### Zoznam nodov

```bash
ros2 node list --no-daemon
```

Pri funkcnej kamera-only pipeline typicky uvidis:

- `/my_camera/basler_cam`
- `/mock_trigger_node`
- `/pairing_node`
- `/dataset_logger_node`

### Zoznam topicov

```bash
ros2 topic list --no-daemon
```

Typicky sa objavi:

- `/my_camera/basler_cam/image_raw`
- `/trigger_pulse`
- dalsie kamerove topicy

### Kontrola datasetu

```bash
ls -lah ~/ros2_tim_projekt/dataset
find ~/ros2_tim_projekt/dataset -maxdepth 2 -type f | head
```

---

## 10. Ako zastavit stack

Ak stack bezi v popredi, staci `Ctrl+C`.

Ak ostanu visiet procesy, pouzi:

```bash
pkill -f pylon_ros2_camera_wrapper
pkill -f mock_trigger_node.py
pkill -f stm32_trigger_node.py
pkill -f pairing_node.py
pkill -f dataset_logger_node.py
```

---

## 11. Co znamena `pairs.csv`

CSV obsahuje stlpce:

- `seq` – poradove cislo triggeru / paru
- `image_file` – nazov ulozeneho obrazka
- `cloud_file` – nazov ulozeneho cloudu
- `trigger_ns` – cas triggeru v ns
- `image_ns` – timestamp obrazka v ns
- `cloud_ns` – timestamp cloudu v ns
- `dt_img_us` – rozdiel medzi triggerom a obrazkom v mikrosekundach
- `dt_cloud_us` – rozdiel medzi triggerom a cloudom v mikrosekundach

Bez LiDARu je bezne:

- `cloud_file` prazdne
- `cloud_ns` prazdne
- `dt_cloud_us` prazdne

---

## 12. Aktualny stav kvality dat

### Co je dobre

Na zaciatku merania sa ukazuje stabilne casovanie obrazkov voci triggeru. Typicke hodnoty `dt_img_us` boli priblizne okolo:

- `-9 000` az `-10 000 us`

To znamena, ze obrazok prisiel priblizne o 9–10 ms skor alebo mal tak oznaceny timestamp voci triggeru. Pri kamera-only testoch je to pouzitelne a dava to zmysel.

### Co este nie je idealne

V dlhsej nahravke sa objavili aj velke odchylky `dt_img_us`, napriklad okolo:

- `+0.96 s`
- `+3.96 s`
- `+4.00 s`

To naznacuje, ze pri dlhsom behu sa pairing alebo logger mohol opozdovat, pripadne sa pouzivali starsie obrazky z fronty alebo vznikal backlog.

Preto:

- **na zakladny test pipeline su data pouzitelne**
- **na finalne hodnotenie synchronizacie este nie**

Pred pouzitim na realny zber dat odporucame:

1. skratit test na kratsie useky
2. po kazdom teste vycistit `dataset/`
3. zoptimalizovat `dataset_logger_node.py`, lebo moze zatazovat CPU
4. po pripojeni realneho hardveru znova overit `dt_img_us` a `dt_cloud_us`

---

## 13. Cistenie datasetu pred novym testom

Odporucane pred kazdym novym meranim:

```bash
rm -rf ~/ros2_tim_projekt/dataset
mkdir -p ~/ros2_tim_projekt/dataset/images
mkdir -p ~/ros2_tim_projekt/dataset/clouds
```

Potom spustit novy test odznova.

---

## 14. Odporucany postup pre tim

### Ked ma niekto iba kameru

```bash
cd ~/ros2_tim_projekt
source /opt/ros/humble/setup.bash
source install/setup.bash
./run_sync_stack.sh mock color
```

### Ked sa testuje iba kamera bez zvysku

```bash
./run_basler.sh color
```

### Ked bude STM32

```bash
./run_sync_stack.sh stm32 color
```

### Ked bude aj LiDAR

- odkomentovat Hesai launch v `run_sync_stack.sh`
- skontrolovat cloud topic
- spustit plny stack

---

## 15. Zhrnutie

Aktualne je projekt pripraveny na:

- kamera-only testovanie
- overenie, ze pipeline funguje
- ukladanie obrazkov a CSV zaznamov

Aktualne este nie je finalne overene:

- realne HW triggerovanie cez STM32
- synchronizacia s Hesai cloudmi
- stabilita timingov pri dlhom zazname

Na bezny timovy test s Basler kamerou je vsak aktualny stav uz pouzitelny.
