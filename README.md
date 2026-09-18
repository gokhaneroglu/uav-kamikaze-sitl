# Autonomous Kamikaze Guidance, 6-DOF Flight Dynamics & HIL/SITL Simulation Environment 🛩️🎯

[![ArduPilot](https://img.shields.io/badge/ArduPilot-Plane_SITL%2FHIL-red?style=flat&logo=ardupilot)](https://ardupilot.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange?style=flat&logo=gazebo)](http://gazebosim.org/)
[![HIL/SITL](https://img.shields.io/badge/Validation-SITL_%26_HIL-blue)](#-donanım-ve-yazılım-çevrimli-doğrulama-sitl--hil)
[![Flight-Tested](https://img.shields.io/badge/Testing-Field_Flight_Verified-success)](#-gerçek-uçuş-testleri-ve-saha-doğrulaması)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Bu depo; arkadan itişli (pusher) ve V-kuyruk (V-tail) aerodinamik konfigürasyonuna sahip sabit kanatlı bir hava aracının (**Mini Talon**) 6 serbestlik dereceli (6-DOF) uçuş mekaniğini, agresif kamikaze dalış/kurtarma güdüm algoritmalarını ve bu sistemlerin **SITL (Yazılım Çevrim)**, **HIL (Donanım Çevrim)** ve **Gerçek Uçuş Testleri** ile doğrulama altyapısını içerir.

---

### 📌 Mühendislik ve Görev Mimarisi

* **Platform Mimarisi:** Arkadan itişli BLDC motor itki dinamiği, ruddervator kontrol yüzeyi mikserleri ve yüksek hücum açılarında kararlılık sağlayan V-kuyruk geometrisi.
* **6-DOF Uçuş Dinamiği & Modelleme:** Gazebo fizik motoru üzerinde hava aracının boyuna ve yanal kararlılık türevleri, atalet momentleri ve aerodinamik sürükleme katsayıları modellenmiştir[cite: 10].
* **Otonom Kamikaze Dalış & Kurtarma (Pull-Up) Kontrolü:** 
  * Yer hedefi (QR kod plakaları) tespit edildiğinde dik açılı terminal dalış fazına geçiş[cite: 10].
  * Dalış esnasında hava hızının ve kanat yükünün sınır değerleri aşmaması için kapalı çevrim irtifa/açı PID denetimi[cite: 10].
  * Hedef taramasının ardından uçağın zemin çarpışmasını önleyen ve sanal koridorda tutan ani irtifa kurtarma (pull-up) manevra dinamiği[cite: 10].
* **Sensör Füzyonu & Kestirim:** IMU, GNSS, Pitot tüpü (hava hızı) ve barometre verilerinin Extended Kalman Filter (EKF) ile birleştirilerek dalış fazında yönelim (Euler/Kuaterniyon) sapmalarının sıfırlanması[cite: 10].
* **Hava Sahası Güvenliği (No-Fly Zone):** Sanal poligonda uçuş koridoru ihlalini engelleyen dinamik yasaklı alan sınırlandırma algoritmaları[cite: 10].

---

### 🔬 Donanım ve Yazılım Çevrimli Doğrulama (SITL & HIL)

Platformun kontrol ve güdüm yazılımları sahaya çıkmadan önce iki kademeli test çevriminden geçirilmiştir[cite: 10]:

1. **Yazılım Döngüde Simülasyon (SITL):**
   * Gazebo dünyasında tekli (`vtail_runway.sdf`) ve çift uçaklı (`vtail_runway2.sdf`) hava muharebesi ortamı[cite: 10].
   * Dalış ve optik tespit için 5 farklı noktaya yerleştirilmiş QR kod hedef plakaları ve rüzgar/türbülans parametreleri[cite: 10].
2. **Donanım Döngüde Simülasyon (HIL):**
   * Gerçek uçuş kartı (**Pixhawk / Cube Orange**), USB/FTDI arayüzü üzerinden simülasyon bilgisayarına bağlanarak ArduPlane HIL modunda koşturulmuştur[cite: 10].
   * Gazebo'dan üretilen sanal sensör verileri (IMU ham verisi, hava hızı, GNSS) doğrudan fiziksel otopilot işlemcisine beslenmiş; otopilotun ürettiği servo/PWM çıkışları gerçek zamanlı olarak Gazebo aerodinamik yüzeylerine geri iletilmiştir[cite: 10].

---

### 🛫 Gerçek Uçuş Testleri ve Saha Doğrulaması

* **Telemetri & Log Analizi:** SITL ve HIL ortamlarında optimize edilen PID katsayıları (`mini_talon_vtail.param`), fiziksel platforma yüklenerek pistten otonom kalkış, hat takibi ve dalış manevraları gerçekleştirilmiştir[cite: 10].
* **Model Doğrulama:** Uçuş testlerinden elde edilen `.bin` veri logları (pitch/roll sapması, irtifa kaybı, hava hızı) simülasyon çıktıları ile üst üste bindirilerek kontrolcü yanıtları doğrulanmıştır[cite: 10].

---

### 📂 Dizin Yapısı

```text
combat-uav-gazebo-sitl/
├── gazebo/
│   ├── models/
│   │   ├── mini_talon_vtail/     # Arkadan itişli V-kuyruk İHA fiziksel ve aerodinamik modeli
│   │   ├── mini_talon_vtail2/    # Çift araçlı senaryolar için ikincil hedef/rakip İHA
│   │   ├── qr_code/ .. qr_code5/ # Kamikaze dalışı hedef plakaları
│   │   ├── runway/               # 3D modellenmiş kalkış/iniş pisti
│   │   └── sun/                  # Çevresel ışık ve gölge modeli
│   └── worlds/
│       ├── vtail_runway.sdf      # Tekli araç dalış ve görev pisti dünyası
│       └── vtail_runway2.sdf     # Çiftli araç ve dinamik hava sahası dünyası
├── SITL_Models/
│   └── Gazebo/
│       └── config/
│           ├── mini_talon_vtail.param   # Doğrulanmış ArduPlane aerodinamik ve PID kazançları
│           └── mini_talon_vtail2.param  # 2. İHA otopilot parametre konfigürasyonu
└── README.md
