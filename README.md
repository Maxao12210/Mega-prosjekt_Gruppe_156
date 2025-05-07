# Mega-prosjekt_Gruppe_156

Denne README-en beskriver hvordan du bygger, kjører og kalibrerer kamerasystemet for `cube_pointer_robot`-pakken på ROS 2 Jazzy.

---

## Forutsetninger

- Ubuntu 22.04 (Noble) eller tilsvarende  
- ROS 2 Jazzy installert  
- Webkamera koblet til `/dev/video1` (bruker må være i `video`-gruppen)  
- Nødvendige ROS-pakker og verktøy:

  ```bash
  sudo apt update
  sudo apt install \
    ros-jazzy-v4l2-camera \
    ros-jazzy-camera-info-manager \
    ros-jazzy-rqt-image-view \
    ros-jazzy-camera-calibration \
    xwayland
Klargjøre workspace

cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

Kjøre kamera og fargedeteksjon

    Sett XCB som Qt-plattform (unngå Wayland-problemer):

export QT_QPA_PLATFORM=xcb

Start kamerasystemet:

    ros2 launch cube_pointer_robot camera_system.launch.py

        Publiserer bilder til /image_raw

        Publiserer kamerainfo til /camera/camera_info

        Starter fargedeteksjonsnodene som abonnerer på /image_raw

Se live-bilder

I en ny terminal (igjen med Qt-plattform og sourced workspace):

export QT_QPA_PLATFORM=xcb
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view

    Velg topic /image_raw

    Sett QoS reliability til Best effort

Kamerakalibrering

    Forbered terminal for kalibrering:

export QT_QPA_PLATFORM=xcb
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

Start kalibrering:

ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.024 \
  --ros-args \
    -r image:=/image_raw \
    -r camera:=/camera

I GUI:

    Velg Checkerboard-modus

    Trykk b gjentatte ganger for å samle ≥25 prøver

    Når du er klar, klikk Calibrate

    Du skal se:

        Wrote calibration data to /tmp/calibrationdata.tar.gz

Pakk ut YAML-fil

mkdir -p ~/.ros/camera_info
tar -xvf /tmp/calibrationdata.tar.gz \
    --strip-components=1 \
    -C ~/.ros/camera_info

Verifisere kalibrering

ros2 topic echo /camera/camera_info --once

Du skal nå se de kalibrerte K, D, R og P-matrisene i meldingen.
