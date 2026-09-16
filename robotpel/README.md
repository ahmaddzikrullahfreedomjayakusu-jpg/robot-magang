# robotpel

Coverage navigation (boustrophedon/lawnmower) untuk robot pel hoverboard, dibangun
di atas SLAM (mapping) + AMCL (localization) + Nav2 (navigasi & obstacle avoidance).

Paket ini hanya menangani sisi ROS 2/Nav2. Koneksi ke motor STM32/hoverboard
tetap lewat node yang sudah ada di folder induk (`robot1.py` / `robotmaganglidar1.py`),
yang mem-publish `/odom` dan mendengarkan `/motor_rpm` lalu meneruskannya lewat
TCP ke HP bridge -> USB OTG -> STM32. `cmd_vel_to_motor_bridge` di paket ini
hanya menerjemahkan output Nav2 (`/cmd_vel`) ke format `/motor_rpm` yang sudah
dipahami node tersebut -- tidak membuka koneksi baru ke hardware.

## Isi paket

- `robotpel/coverage_planner_node.py` -- baca `/map` + `/amcl_pose`, susun jalur
  boustrophedon dari free space, kirim tiap titik sebagai goal Nav2
  (`NavigateToPose`), lacak area yang sudah dipel di grid, dan deteksi kapan
  ruangan sudah selesai (termasuk fase "mop-up" untuk sel yang terlewat).
- `robotpel/cmd_vel_to_motor_bridge.py` -- ubah `/cmd_vel` (Twist) dari Nav2
  menjadi `/motor_rpm` (Int16MultiArray `[kiri, kanan]`, 127=stop) sesuai
  protokol yang sudah dipakai `robot1.py`.
- `config/coverage_params.yaml` -- parameter operasional (lebar pel, margin
  dinding, threshold rintangan, dll), bisa diubah tanpa recompile.
- `config/nav2_params.yaml` -- parameter Nav2 (AMCL, DWB controller, NavFn
  planner, costmap) dituning untuk footprint ~0.40m x 0.50m.
- `launch/coverage_launch.py` -- launch RPLidar + TF LiDAR + AMCL + Nav2 +
  coverage_planner_node + cmd_vel_to_motor_bridge, load kedua file parameter
  di atas.
- `launch/mapping_launch.py` -- launch RPLidar + TF LiDAR + `slam_toolbox`
  untuk fase mapping (dipakai sebelum `coverage_launch.py` ada map untuk
  dipakai).
- `scripts/start.sh mapping|coverage` -- satu command buat nyalain `robot1.py`
  + launch file yang sesuai sekaligus (kecuali RViz -- dibuka manual, lihat
  di bawah). Edit variabel kalibrasi LiDAR di bagian atas file sebelum
  dipakai.
- `rviz/mapping.rviz`, `rviz/coverage.rviz` -- config RViz siap pakai
  (`rviz2 -d rviz/mapping.rviz`), display sudah ke-set semua (termasuk
  marker badan robot dari `/robot_body`), tidak perlu
  klik "Add" manual.

## 0. Instalasi dependency (sekali saja)

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
                  ros-jazzy-slam-toolbox ros-jazzy-rplidar-ros \
                  python3-numpy
```

Sesuaikan `ros-jazzy-*` dengan distro ROS 2 kamu bila bukan Jazzy.

## 1. Build

Taruh folder `robotpel` di dalam `src/` workspace colcon, misalnya:

```bash
mkdir -p ~/ros2_ws/src
ln -s "/home/freedom/Documents/Robot magang/robotpel" ~/ros2_ws/src/robotpel
cd ~/ros2_ws
colcon build --packages-select robotpel
source install/setup.bash
```

## 2. Nyalakan hardware dulu

RPLidar sekarang sudah dibundel di dalam `mapping_launch.py` (fase mapping)
dan `coverage_launch.py` (fase ngepel) -- tidak perlu dinyalakan terpisah
lagi. Yang perlu dinyalakan manual di sini cuma dua:

```bash
# bridge STM32 (di HP/Android, sudah ada): stm_bridge.py

# node robot: publish /odom + TF odom->base_footprint, dengarkan /motor_rpm
python3 "/home/freedom/Documents/Robot magang/robot1.py"
```

Cek `ros2 topic hz /odom` sebelum lanjut.

### Kalibrasi TF LiDAR (WAJIB, sekali per pemasangan)

`robot1.py` cuma mem-publish `odom -> base_footprint`. Tidak ada apa pun di
stack yang ada sekarang yang mem-publish transform dari `base_footprint` ke
frame LiDAR -- tanpa ini SLAM/Nav2 tidak bisa menaruh data `/scan` di
map/costmap sama sekali (map bakal kosong / robot jalan tanpa mendeteksi
rintangan apa pun). Kedua launch file (`mapping_launch.py` dan
`coverage_launch.py`) sudah menyertakan node `static_transform_publisher`
untuk transform ini, tapi nilainya masih placeholder dan HARUS diisi sesuai
robot asli. Cek dulu frame default RPLidar kamu (biasanya `laser`):

**Catatan arah:** default `laser_yaw` sudah di-set ke `3.14159` (180 derajat),
BUKAN `0`. Ini karena LiDAR di robot ini terpasang menghadap ke arah
sebaliknya dari arah jalan robot (arah "maju" robot = sisi belakang LiDAR).
Kalau ini dibiarkan `0`, Nav2 akan salah baca posisi rintangan (yang di
depan dikira di belakang, dan sebaliknya) -- bisa bikin robot nabrak. Kalau
ternyata pemasangan LiDAR kamu beda lagi (miring, menyamping, dll), sesuaikan
`laser_yaw` sampai arah 0° pembacaan `/scan` benar-benar cocok dengan arah
maju `base_footprint` (cek lewat RViz: bandingkan posisi rintangan asli
dengan titik LaserScan yang muncul).

**Catatan lain:** default `laser_inverted` juga sudah di-set `true` (bukan
`false`), karena LiDAR di robot ini kepasang terbalik (upside-down) --
gejalanya kalau salah: kiri/kanan hasil scan/map ke-mirror dibanding dunia
nyata (belok kiri malah nambah map di sisi kanan, dst). Kalau ternyata masih
mirror setelah ini, coba balik jadi `laser_inverted:=false`.

```bash
ros2 launch robotpel mapping_launch.py serial_port:=/dev/ttyUSB0
# di terminal lain:
ros2 topic echo /scan --field header.frame_id
```

Lalu pasang nilai offset yang benar tiap kali launch (samakan nilainya di
`mapping_launch.py` maupun `coverage_launch.py`, karena keduanya menjelaskan
posisi fisik LiDAR yang sama):

```bash
ros2 launch robotpel mapping_launch.py \
    serial_port:=/dev/ttyUSB0 \
    laser_frame:=<hasil dari header.frame_id di atas> \
    laser_x:=<offset x LiDAR ke pusat robot, meter> \
    laser_y:=<offset y> \
    laser_z:=<tinggi LiDAR dari lantai> \
    laser_yaw:=<jika LiDAR terpasang menghadap bukan ke depan robot>
```

## 3. Mapping ruangan (SLAM)

Command yang sama seperti kalibrasi TF di atas -- `mapping_launch.py` sudah
menyalakan RPLidar + TF LiDAR + `slam_toolbox` sekaligus dalam satu launch
(dan sudah memaksa `use_sim_time:=false`, karena launch file bawaan
`slam_toolbox` defaultnya `true` dan akan macet diam di robot asli):

```bash
ros2 launch robotpel mapping_launch.py \
    serial_port:=/dev/ttyUSB0 \
    laser_frame:=laser \
    laser_z:=0.15   # dst, isi sesuai kalibrasi di atas

rviz2   # tambahkan display /map untuk lihat hasil mapping live
```

Gerakkan robot secara manual (pakai controller manual yang sudah ada di
`robotmaganglidar1.py`/`robot1.py`) berkeliling ruangan sampai seluruh area,
termasuk sekitar meja/kursi, sudah ter-mapping dengan baik di RViz.

## 4. Simpan map

```bash
mkdir -p "/home/freedom/Documents/Robot magang/robotpel/maps"
ros2 run nav2_map_server map_saver_cli \
    -f "/home/freedom/Documents/Robot magang/robotpel/maps/room" \
    --ros-args -p save_map_timeout:=5.0
```

Ini menghasilkan `room.yaml` + `room.pgm`. Matikan `slam_toolbox` setelah ini
(coverage run memakai AMCL, bukan SLAM).

## 5. Jalankan coverage

Pakai `laser_frame`/`laser_x/y/z/roll/pitch/yaw` dan `serial_port` yang SAMA
seperti yang sudah dikalibrasi di langkah 2 (posisi fisik LiDAR-nya kan tidak
berubah):

```bash
ros2 launch robotpel coverage_launch.py \
    map:="/home/freedom/Documents/Robot magang/robotpel/maps/room.yaml" \
    serial_port:=/dev/ttyUSB0 \
    laser_frame:=laser \
    laser_z:=0.15
```

Di RViz2:

1. Set `2D Pose Estimate` di posisi awal robot yang sebenarnya (AMCL butuh
   initial pose manual karena `set_initial_pose: false`).
2. Tunggu `coverage_planner_node` log "Boustrophedon plan ready: N waypoints"
   -- ini muncul setelah AMCL cukup yakin dengan posisinya
   (`require_localized`/`max_pose_covariance` di `coverage_params.yaml`).
3. Robot akan berjalan lajur demi lajur; Nav2 sendiri yang menangani
   penghindaran rintangan dan belokan U-turn antar lajur lewat costmap.

Pantau progres:

```bash
ros2 topic echo /coverage/status     # WAITING_MAP / NAVIGATING / MOPPING_UP / FINISHED
ros2 topic echo /coverage/percent    # persen area terpel
ros2 topic echo /coverage/complete   # true saat selesai
```

Tambahkan display `/coverage/grid` (OccupancyGrid) di RViz untuk melihat
langsung sel mana yang sudah/belum terpel.

## Kalibrasi wajib sebelum dipakai di robot asli

- `cmd_vel_to_motor_bridge`: `wheel_base` dan `max_linear_speed` di
  `coverage_params.yaml` masih perkiraan -- uji jalan lurus dan putar di
  tempat, sesuaikan sampai gerakan robot sesuai perintah Nav2.
- `robot_half_width`, `wall_margin`, footprint di `nav2_params.yaml` -- ukur
  badan robot + kain pel yang sebenarnya.
- `mop_width` -- sesuaikan dengan lebar pel asli (default 0.40m).

## Keterbatasan yang perlu diketahui

- Planning boustrophedon hanya dilakukan sekali dari map pertama yang
  diterima (`on_map` mengabaikan map berikutnya). Restart node kalau mau
  merencanakan ulang dari map baru.
- Fase mop-up mengunjungi sel terlewat satu-satu berdasarkan jarak terdekat
  ke posisi robot saat itu, tanpa mengelompokkan pocket kecil -- cukup untuk
  ruangan berukuran wajar, tapi bisa lambat kalau area terlewat sangat
  terpecah-pecah.
- `coverage_params.yaml` dan `nav2_params.yaml` tidak saling sinkron secara
  otomatis: mengubah `mop_width`/`wall_margin` di `coverage_params.yaml`
  TIDAK ikut mengubah `footprint`/`inflation_radius` di `nav2_params.yaml`.
  Kalau salah satu diubah, cek juga yang lain.
- `navigation2`/`nav2-bringup` sudah terpasang dan `colcon build` +
  `ros2 launch ... --show-args` sudah dicek jalan tanpa error, tapi ini baru
  memastikan launch file & param file valid secara sintaks/struktur -- belum
  pernah benar-benar dites jalan di robot fisik (belum ada RPLidar/hoverboard
  yang tersambung ke mesin dev ini). Kemungkinan masih ada penyesuaian kecil
  begitu dicoba di robot asli.
- Tidak ada penanganan rintangan rendah/kaca yang tidak kena LiDAR (mis. kaki
  kursi tipis di bawah bidang scan) -- kalau ini masalah nyata di lapangan,
  perlu sensor tambahan + `RangeSensorLayer` seperti di `PEL/mopping_nav`.
