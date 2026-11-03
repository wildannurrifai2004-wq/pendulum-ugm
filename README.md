# Inverted Pendulum UGM - Versi Tanpa Massa (Energi Ternormalisasi)

Folder ini merupakan modifikasi dari `pendulum-wildan-oke` yang telah menerapkan **Normalisasi Error Energi (*Scale-Invariant Energy Control*)**.

---

## 🎯 Tujuan Perbaikan
Menghilangkan ketergantungan nilai $k_E$ terhadap massa batang ($m$) dan variasi panjang batang.
* **Sebelumnya:** Batang panjang ($l = 0.1925\text{ m}$) butuh $k_E \approx 50$, sedangkan batang pendek ($l = 0.0425\text{ m}$) butuh $k_E \approx 1000 - 1200$.
* **Setelah Perbaikan:** Baik batang panjang maupun batang pendek menggunakan rentang $k_E$ yang seragam (sekitar **30 – 80**).

---

## 📝 Lokasi & Detail Perubahan
File yang dimodifikasi:  
`Pendulum_18 JULI/Include/lipmcontrol.h` pada fungsi `swingup_controller()`:

```c
// Target energi posisi tegak atas
float E_des = 2.0f * m_p * g * l_p * gSwing.margin_E;

// Normalisasi error energi relatif terhadap E_des (massa m_p saling membagi habis)
float E_tilde = (E_des > 0.001f) ? ((E - E_des) / E_des) : 0.0f;

// Gain energi ternormalisasi
float kE = gSwing.kE; 

if(degree>150 || degree<-150){
    if(theta_dot_rad * cosf(theta_rad)>0){
        u_energy = kE * E_tilde * 1.0f * fabs(cosf(theta_rad));
    }else{
        u_energy = kE * E_tilde * -1.0f * fabs(cosf(theta_rad));
    }
}
```

---

## 🚀 Cara Menjalankan

### 1. Flash Firmware ke STM32:
* Buka proyek Keil uVision / STM32CubeIDE di dalam folder `Pendulum_18 JULI/`.
* Rebuild proyek dan flash file `.hex` / `.bin` ke mikrokontroler STM32F4.

### 2. Jalankan GUI Python:
* Double click file `JALANKAN_TANPA_MASSA.bat` di root directory.
* Atau jalankan secara manual lewat terminal:
  ```bash
  cd "pendulum tanpa massa/GUI Pendulum"
  python pendulum-monitoring.py
  ```

### 3. Penggunaan di GUI:
* **Panjang Pendulum ($l$):** Wajib diukur dan diinput sesuai jarak pivot ke Center of Mass (CoM).
* **Massa Pendulum ($m$):** Biarkan pada nilai default (tidak perlu ditimbang ulang).
* **Gain $k_E$:** Mulai uji coba dari rentang **30**, naikkan bertahap (misal 35, 40, 45, 50) hingga batang terayun pas ke titik puncak.
