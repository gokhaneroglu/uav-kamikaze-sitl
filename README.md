# Combat UAV Autonomous Guidance & Gazebo SITL Simulation 🛩️

[![ArduPilot](https://img.shields.io/badge/ArduPilot-Plane_SITL-red?style=flat&logo=ardupilot)](https://ardupilot.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange?style=flat&logo=gazebo)](http://gazebosim.org/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?style=flat&logo=ros)](https://docs.ros.org/)
[![Blender](https://img.shields.io/badge/Blender-3D_Modeling-darkorange?style=flat&logo=blender)](https://www.blender.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Bu depo, **TEKNOFEST Savaşan İHA** yarışması senaryoları için geliştirilmiş; sabit kanatlı **Mini Talon V-Tail** hava araçlarının 6-DOF uçuş dinamiklerini, otonom güdüm/hedefleme algoritmalarını ve hava-hava/hava-yer muharebe senaryolarını test eden kapsamlı **Gazebo + ArduPilot SITL** simülasyon altyapısını içerir.

---

### 🚀 Temel Mühendislik Yetkinlikleri

* **Çift Platformlu (Dual-UAV) Simülasyon:** İki adet Mini Talon V-Tail sabit kanat İHA'nın eşzamanlı uçuş, formasyon ve it dalaşı (dogfight) senaryolarını destekleyen çoklu araç altyapısı (`vtail_runway.sdf`, `vtail_runway2.sdf`).
* **GNSS Lock & Optik Takip Geçişi:** Rakip hava araçlarına önce GNSS telemetrisi üzerinden kilit atma, ardından burun kamerasına geçerek görüntü işleme tabanlı sürekli optik takibe geçiş sağlayan hibrit güdüm mimarisi.
* **QR Kod Dalış & Kurtarma PID Kontrolü:** Yerdeki QR kod hedef plakalarına (`qr_code` 1-5) agresif açıyla dalış (kamikaze manevrası) yaparken uçağın sanal koridordan çıkmamasını sağlayan kapalı çevrim irtifa/açı PID kontrolü ve ani irtifa kurtarma (pull-up) dinamikleri.
* **Dinamik No-Fly Zone (Yasaklı Alan Kaçınması):** Yarışma sahası sınırlarının ihlal edilmesini önleyen dinamik sanal sınır koruma ve çarpışma önleme algoritmaları.
* **3D Çevre ve Parkur Modellemesi:** Yarışma pisti (`runway`), zemin sürtünme modelleri ve hedef plakaları Blender ortamında 1:1 ölçekli tasarlanarak Gazebo SDF formatına optimize edilmiştir.
* **ArduPlane Parametre Kalibrasyonu:** Mini Talon V-Tail gövde geometrisi, aerodinamik kontrol yüzeyleri ve fırçasız (BLDC) motor itki sistemine özel optimize edilmiş `.param` yapılandırmaları.

---

### 📂 Dizin Yapısı

```text
combat-uav-gazebo-sitl/
├── gazebo/
│   ├── models/
│   │   ├── mini_talon_vtail/     # Birincil Mini Talon V-Tail modeli ve sensör eklentileri
│   │   ├── mini_talon_vtail2/    # İkincil rakip/hedef İHA modeli
│   │   ├── qr_code/ .. 5/        # Dalış ve görüntü işleme için yer hedef plakaları
│   │   ├── runway/               # Kalkış ve iniş pisti 3D modeli
│   │   └── sun/                  # Aydınlatma ve güneş konfigürasyonu
│   └── worlds/
│       ├── vtail_runway.sdf      # Tek araçlı görev ve pist simülasyon dünyası
│       └── vtail_runway2.sdf     # Çift araçlı hava muharebesi simülasyon dünyası
├── SITL_Models/
│   └── Gazebo/
│       └── config/
│           ├── mini_talon_vtail.param   # 1. İHA ArduPlane kazanç ve uçuş parametreleri
│           └── mini_talon_vtail2.param  # 2. İHA ArduPlane kazanç ve uçuş parametreleri
└── README.md
