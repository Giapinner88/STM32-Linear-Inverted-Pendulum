# RUNBOOK - STM32 Linear Inverted Pendulum

Muc tieu: test nhanh, an toan, lap lai duoc. Sau khi pass runbook nay se chuyen sang Energy Shaping + LQR.

## 1. Pre-flight (1-2 phut)

- Dat xe giua hanh trinh, tranh sat 2 dau ray.
- Con lac o trang thai tha xuong.
- Kiem tra nguon on dinh, day ST-Link chac chan.
- Kiem tra nut:
  - USER: RUN/STOP
  - M: SET ZERO
  - + / -: trim manual torque khi chua lock upright
- Build va upload:
  - `~/.platformio/penv/bin/pio run -t upload`

## 2. Boot sanity check

- Sau reset, LED phai nhay boot pattern.
- OLED hien man hinh swing-up status.
- Neu OLED trang hoac treo: dung test, kiem tra cap nguon/OLED wiring truoc.

## 3. Zero calibration (bat buoc)

- Tu xoay con lac de ADC nam trong cua so 1010-1030 (moc tha xuong).
- Bam nut M de set zero.
- Xac nhan tren OLED:
  - `Z` duoc cap nhat.
  - Trang thai `OK` (neu firmware dang hien thi `OK/ER`).
- Neu `ER`: tiep tuc xoay ve dung cua so roi bam M lai.

## 4. Chay swing-up

- Bam USER de chuyen RUN (`R=1`).
- Khi chua lock upright (`C=0`), dung nut `+/-` de trim `MU` (manual torque) nham test huong dong co, encoder va truyen thong command.
- Quan sat:
  - `U%` thay doi theo chuyen dong, khong dung cung mot gia tri.
  - `MU` thay doi khi bam `+/-`.
  - `RL=1` nghia la dang bi chan mem cuoi hanh trinh (soft rail guard), firmware se cat torque.
  - Con lac dao dong va tang bien do dan.
  - Khi gan dinh, co vao pha capture (`C=1`) neu du dieu kien.

## 5. Dung khan cap va an toan

- Bam USER bat ky luc nao de ve STOP (`R=0`).
- Neu xe lao sat ray, dung ngay bang USER.
- Moi lan doi tham so lon, dat xe ve giua ray truoc khi RUN lai.

## 6. Checklist pass cho giai doan hien tai

- [ ] Zero set on dinh trong cua so 1010-1030
- [ ] USER RUN/STOP phan hoi dung
- [ ] U% khong bang-bang cung mot muc co dinh
- [ ] Con lac swing duoc qua lai co kiem soat
- [ ] Khong lao ray lap lai do loi logic

## 7. Logging toi thieu moi lan test

Ghi vao DEBUG_LOG.md:
- Timestamp
- Z (ADC zero)
- Hanh vi U% (muot / giat)
- Co vao capture khong
- Ly do stop (thu cong / sat ray / mat on dinh)
- Tham so da doi (neu co)

## 8. Chuyen tiep sang Energy Shaping + LQR

Tien dieu kien:
- Sensor angle on dinh, zero calibration lap lai duoc.
- RUN/STOP tin cay.
- Swing-up thuan co huong dung, khong rail lock.

Ke hoach ngay sau runbook:
1. Co dinh energy shaping (swing phase) voi bo tham so an toan.
2. Linearize quanh upright va nap gain LQR (capture/balance phase).
3. Them hysteresis chuyen mode:
   - vao LQR khi |theta - pi| nho va |dtheta| nho
   - quay lai swing-up neu roi khoi mien on dinh.
4. Ghi log theo tung bo gain de chot bo tham so cuoi.
