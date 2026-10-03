# manual_motor_test_cdx

Project kecil untuk test manual motor hoverboard via STM32 serial.

Tidak memakai ROS, LiDAR, RViz, YOLO, atau program robot besar.

## Jalankan

Jalankan langsung, port STM32 akan auto-detect dari USB VID:PID `0483:5740`:

```bash
./start_manual_motor.sh
```

Atau:

```bash
python3 manual_motor_test.py
```

Kalau suatu saat ingin paksa port manual:

```bash
python3 manual_motor_test.py --port /dev/ttyACM1
```

## Kontrol

- Tahan `panah atas`: maju dua roda `[97, 97]`
- Tahan `panah bawah`: mundur dua roda `[157, 157]`
- Tahan `panah kiri`: spin kiri `[157, 97]`
- Tahan `panah kanan`: spin kanan `[97, 157]`
- `x` atau spasi: stop `[127, 127]`
- `Ctrl+C`: keluar aman dan stop motor

Saat tombol panah dilepas, script otomatis mengirim stop setelah timeout pendek.
