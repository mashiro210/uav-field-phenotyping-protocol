# 🚁 Field Phenomics UAV Mission Suite & Protocols

This repository provides a comprehensive suite of Python-based tools for generating advanced custom UAV flight missions (KMZ files), alongside the standard operational protocols for high-precision field phenomics and 3D reconstruction.

## 📖 Comprehensive Operational Protocols (Wiki)
For detailed field operations, hardware setups, and network configurations, **please refer to our Wiki.**
It covers everything from basic GCP installation to full off-grid automation using Starlink and DJI Docks.

👉 **[Read the Field Phenomics UAV Protocols Wiki Here](https://github.com/mashiro210/uav-field-phenotyping-protocol/wiki)**

---

## 🚀 Key Features of the Mission Generator (Python Suite)
The Python scripts included in this repository allow you to bypass the limitations of standard DJI flight apps, enabling complex, highly customized data acquisition tailored for field phenomics.

**Supported Hardware:**
* DJI Mavic 3 Multispectral (M3M)
* DJI Matrice 300 / 350 / 400 RTK + Zenmuse P1

**Available Mission Templates:**
* **Ultra-Low POI One-Shot:** Targeted missions that navigate to predefined Points of Interest (POI), hover, and capture a single high-precision shot before moving to the next.
* **Arbitrary Altitude Mapping:** Custom grid mapping missions that can be set to any specific altitude requirements.
* **CCO (Cross Circle Oblique) & Dome CCO:** Automated cylindrical and dome-shaped flight paths focusing on specific targets (e.g., individual tree crowns or complex structures) for flawless 3D model reconstruction.
* **Linear Interval Shooting:** Flight paths designed to follow specific transects or crop rows, capturing images at precise distance or time intervals.

---

