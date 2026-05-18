CSV:
seq,cam_hw_ts_ns,host_ts_ns,block_id,width,height,line_status_all,image_path

seq = poradové číslo snímky
cam_hw_ts_ns = timestamp z kamery
host_ts_ns = timestamp z PC v momente spracovania
block_id = id frame-u z kamery
image_path = cesta k fotke

Čiže už teraz vieš ku každej fotke priradiť timestamp.

!!!!!!Čo odporúčam tebe na začiatok!!!!!

Na prvé fungujúce párovanie používaj:

kamera: host_ts_ns
lidar: host timestamp z PC, keď prišiel packet/cloud/message

Až neskôr, keď budeš mať presne vyriešenú synchronizáciu a offsety, môžeš riešiť:

kamera cam_hw_ts_ns
lidar sensor timestamp

Čo by som ešte pridal do CSV, aby sa ti lepšie párovalo

Teraz už tam máš skoro všetko potrebné, ale ja by som ti odporúčal pridať aj:

host_time_sec v čitateľnom tvare
alebo aspoň ros_stamp_sec
prípadne filename

Ale to nie je nutné, lebo image_path tam už máš.

Na prvé testovanie ti úplne stačí súčasný CSV.

Ako si potom predstavovať párovanie s LiDARom

Predstav si, že máš:

Kamera CSV: 
seq,cam_hw_ts_ns,host_ts_ns,image_path
1,52796610960,1774464283818991468,.../img_000001.png
2,52846668280,1774464283843847448,.../img_000002.png
3,52896725810,1774464283905463706,.../img_000003.png

LiDAR log:
lidar_seq,host_ts_ns,pcd_path
101,1774464283817000000,.../cloud_101.pcd
102,1774464283868000000,.../cloud_102.pcd
103,1774464283912000000,.../cloud_103.pcd

Pre každú fotku nájdeš LiDAR záznam s najbližším host_ts_ns.

Čiže:

img_000001 -> cloud s najbližším host timestamp
img_000002 -> cloud s najbližším host timestamp

Čo znamená mmap v tvojom prípade

Mmap je len rýchly spoločný priestor v RAM, aby si nemusel stále čítať súbor.

Tvoj camera node už zapisuje do:

/dev/shm/liv_sync_ring.bin

a aj do:

/dev/shm/liv_sync_stamp

To znamená:

kamera po každom frame zapíše timestamp a metadata do shared memory
pairing node si to môže okamžite prečítať
nemusí čakať na disk

Na debug a kontrolu je ale CSV stále výborný, lebo je ľahko čitateľný.

Najpraktickejší workflow pre teba teraz

Sprav toto:

A. nechaj ukladať fotky

už funguje

B. nechaj ukladať CSV

už funguje:

/home/dominik/basler_trigger_output_cpp/capture_log.csv
C. pridaj terminal log

podľa úpravy run_basler

D. pri LiDARe tiež zapisuj
host_ts_ns
cestu k súboru alebo id message
E. páruj podľa host_ts_ns

na začiatok je to najjednoduchšie a najrozumnejšie

