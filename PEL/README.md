# PEL - Autonomous Mopping Robot Coverage Navigation

Arsitektur ini fokus pada sisi ROS 2/Nav2 untuk robot ngepel lorong padat barang dengan kaca bawah. Protokol komunikasi motor/sensor sengaja tidak dibahas.

## Struktur Paket

- `mopping_nav/coverage_grid_tracker.py`
  Membuat occupancy matrix cakupan pel dari `/map` dengan resolusi 10 cm. Sel bernilai `0` berarti belum dipel, `100` berarti sudah dipel, dan `-1` berarti bukan lantai bebas/unknown.

- `mopping_nav/corridor_shuttle_bug2.py`
  Controller lorong: wall-following shuttle, Bug-2 saat ada kursi/meja, dan state machine putar balik.

- `mopping_nav/missed_area_dispatcher.py`
  Mode pembersihan ulang: mencari sel `0` terdekat pada `/mopping/coverage_grid`, lalu mengirim goal ke Nav2 `NavigateToPose`.

- `config/nav2_mopping_costmap.yaml`
  Contoh konfigurasi costmap Nav2 dengan inflation radius untuk menjadikan kaki kursi padat sebagai pulau rintangan yang lebih aman.

- `config/mopping_params.yaml`
  Parameter operasi robot: lebar pel, overlap, panjang lorong, jarak dinding, dan threshold rintangan.

## 1. Grid-Based Occupancy Matrix

Costmap/OccupancyGrid dipakai sebagai representasi lantai. Untuk coverage tracking, resolusi diset `0.10` m:

```yaml
coverage_grid_tracker:
  ros__parameters:
    coverage_resolution: 0.10
    mop_width: 0.42
    mop_length: 0.28
```

Alur logika:

1. Node menerima `/map` dari SLAM atau map server.
2. Map di-resample menjadi grid coverage 10x10 cm.
3. Sel yang bukan area bebas pada static map ditandai `-1`.
4. Setiap periode, node membaca posisi robot `map -> base_link` dari TF.
5. Jejak kain pel dihitung sebagai footprint elips/lingkaran konservatif.
6. Sel yang terkena footprint diubah dari `0` menjadi `100`.
7. Progress dipublish ke `/mopping/coverage_percent`.
8. Sel terlewat terdekat dipublish ke `/mopping/next_missed_cell`.

Metode tracking area sisa:

- Jalankan shuttle coverage sampai selesai.
- Publish `std_msgs/Bool(data=true)` ke `/mopping/cleanup_missed_cells`.
- `missed_area_dispatcher` akan mencari sel bernilai `0` terdekat dan mengirim goal ke Nav2.
- Setelah robot melewati sel tersebut, `coverage_grid_tracker` akan mengubahnya menjadi `100`.

## 2. Wall-Following Shuttle + Bug-2

Mode utama robot adalah menyusuri satu sisi lorong:

- `wall_side: left` berarti robot menjaga jarak terhadap dinding/kaca kiri.
- `desired_wall_distance: 0.35` menjaga badan dan kain pel tetap aman.
- LiDAR dibagi menjadi sektor `front`, `left`, `right`, `front_left`, dan `front_right`.
- Range sensor bumper bawah `/front_low_range` digabung dengan sektor depan LiDAR agar kaca bawah yang tidak terlihat LiDAR tetap dianggap rintangan.

Bug-2:

1. Saat mulai satu lajur, robot menyimpan `lane_start` dan `lane_goal`. Garis ini adalah M-Line.
2. Robot bergerak sepanjang M-Line sambil wall-following.
3. Jika depan tertutup, robot menyimpan posisi/proyeksi hit point lalu masuk `BUG_CIRCUMFILTRATE`.
4. Robot mengikuti kontur rintangan di sisi yang sama sampai:
   - kembali dekat M-Line,
   - sudah lebih maju dari hit point,
   - dan depan kembali aman.
5. Robot kembali ke `FOLLOW_MLINE`.

Inflation radius:

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  inflation_radius: 0.42
  cost_scaling_factor: 2.2
```

Nilai awal yang disarankan:

- `robot_radius`: radius badan robot, misalnya `0.24` m.
- `inflation_radius`: minimal `robot_radius + 0.5 * mop_width + margin`.
- Untuk kaki kursi padat, mulai dari `0.35` sampai `0.50` m. Semakin besar nilainya, celah sempit akan dianggap tertutup sehingga robot tidak memaksa masuk dan kain pel tidak tersangkut.
- Jika robot terlalu jauh dari tepi barang, naikkan `cost_scaling_factor` secara hati-hati atau turunkan `inflation_radius`.

## 3. State Machine Putar Balik

State utama:

- `FOLLOW_MLINE`: jalan lurus mengikuti lorong.
- `BUG_CIRCUMFILTRATE`: mengitari kursi/meja sampai kembali ke M-Line.
- `STOP_AT_BOUNDARY`: berhenti saat batas lajur tercapai.
- `SHIFT_LATERAL`: maju-melengkung/geser sebesar `mop_width - overlap`.
- `TURN_180`: putar 180 derajat.
- `FINISHED`: semua lajur selesai atau lorong terlalu sempit.

Putar balik dipicu oleh salah satu kondisi:

- LiDAR depan membaca ujung dinding asli: `front_lidar < 0.20` m.
- Proyeksi posisi robot pada M-Line mencapai `corridor_length`.
- Dead-end total: depan, kiri, dan kanan sama-sama dekat.

## Cara Build

Letakkan folder `mopping_nav` ini di dalam workspace ROS 2, lalu:

```bash
colcon build --packages-select mopping_nav
source install/setup.bash
ros2 launch mopping_nav mopping_navigation.launch.py
```

Untuk mengaktifkan mode bersih area terlewat:

```bash
ros2 topic pub --once /mopping/cleanup_missed_cells std_msgs/msg/Bool "{data: true}"
```

## Catatan Integrasi Nav2

- Node `corridor_shuttle_bug2` saat ini publish langsung ke `/cmd_vel`. Pada robot nyata, masukkan ke velocity mux atau Behavior Tree action agar collision monitor/Nav2 safety tetap bisa mengambil alih.
- Local costmap harus menerima `/scan` dan sensor kaca bawah `/front_low_range`.
- Untuk kaca floor-to-ceiling, sensor ToF/ultrasonik bawah sebaiknya dipublish sebagai `sensor_msgs/Range`, lalu masuk `RangeSensorLayer`.
- Untuk differential drive hoverboard, pastikan odometry stabil dan TF `odom -> base_link` tidak loncat, karena coverage grid bergantung pada TF.
