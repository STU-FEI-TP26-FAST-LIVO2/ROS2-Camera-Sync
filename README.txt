./run_basler.sh color
./run_basler.sh slam

run_sync_stack.sh
Tento skript spustí:
kameru
trigger node
pairing node
dataset logger
voliteľne Hesai

./run_sync_stack.sh mock
./run_sync_stack.sh stm32

SPUSTIT:
cd ~/ros2_tim_projekt
source /opt/ros/humble/setup.bash
source install/setup.bash

./run_sync_stack.sh mock color

FUNKCNOST1:
Toto už vyzerá dobre a stabilne.

Z toho nového behu:
dt_img_us je skoro stále okolo 24.2–24.5 ms
v taili nemáš žiadne úlety na stovky ms alebo sekundy
image_file sa ukladá správne
cloud_file je prázdne správne, lebo nemáš LiDAR
počet záznamov rastie plynulo až po seq=287
Čiže pre kamera-only + mock trigger test sú tieto dáta v poriadku.

Čo presne to znamená
Momentálne máš stabilný offset:
obraz prichádza približne 24 ms po triggeri
To nie je chyba samo o sebe. Je to len:
oneskorenie kamery,
drivera,
publishnutia správy,
a toho, ako pairing páruje trigger s image timestampom.

Na test pipeline je to úplne OK. Dôležité je, že:
offset je stabilný
nerozbieha sa v čase
nevidno veľký jitter

Záver
Pre tvoj tím môžeš zatiaľ povedať:
build funguje
Basler funguje
mock režimm funguje
dataset sa ukladá správne
CSV je pri kamera-only teste konzistentné
bez STM32 a LiDARu je správne, že cloud stĺpce ostávajú prázdne
Ak chceš, ďalší krok ti môžem pripraviť:
krátky text do README typu „ako interpretovať pairs.csv“
alebo
úpravu Python nodeov, aby po Ctrl+C nekončili tracebackom.

---------------------------------------------------------------------------------
Pre STM verziu vieš zmeniť port aj baudrate takto:
./Lidar_STM.sh color /dev/ttyUSB0 115200
Ak bude STM na inom porte, napríklad:
./Lidar_STM.sh color /dev/ttyACM0 115200

Kamera + LiDAR bez STM:
./Lidar_NO_STM.sh
Plný reálny režim:
./Lidar_STM.sh

Kamera only test: 
./run_sync_stack.sh mock color

