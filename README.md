% README for GitHub
\documentclass[a4paper,11pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage[norsk]{babel}
\usepackage{hyperref}
\usepackage{geometry}
\geometry{margin=2.5cm}
\usepackage{listings}
\lstset{
  basicstyle=\ttfamily\small,
  breaklines=true
}

\title{Mega-prosjekt\_Gruppe\_156\\[1ex]
\large README: Bygg, kjøring og kalibrering av kamerasystem for \texttt{cube\_pointer\_robot}}
\author{}
\date{}

\begin{document}
\maketitle

\section*{Forutsetninger}
\begin{itemize}
  \item Ubuntu 22.04 (Noble) eller tilsvarende
  \item ROS 2 Jazzy installert
  \item Webkamera koblet til \texttt{/dev/video1} (bruker må være i \texttt{video}-gruppen)
  \item Nødvendige ROS-pakker og verktøy:
\end{itemize}

\begin{lstlisting}[language=bash]
sudo apt update
sudo apt install \
  ros-jazzy-v4l2-camera \
  ros-jazzy-camera-info-manager \
  ros-jazzy-rqt-image-view \
  ros-jazzy-camera-calibration \
  xwayland
\end{lstlisting}

\section*{Klargjøre workspace}
\begin{lstlisting}[language=bash]
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
\end{lstlisting}

\section*{Kjøre kamera og fargedeteksjon}
\begin{lstlisting}[language=bash]
export QT_QPA_PLATFORM=xcb
ros2 launch cube_pointer_robot camera_system.launch.py
\end{lstlisting}

Publiserer bilder til \texttt{/image\_raw} og kamerainfo til \texttt{/camera/camera\_info}, samt starter fargedeteksjonsnoden som abonnerer på \texttt{/image\_raw}.

\section*{Se live-bilder}
\begin{lstlisting}[language=bash]
export QT_QPA_PLATFORM=xcb
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
\end{lstlisting}

\begin{enumerate}
  \item Velg topic \texttt{/image\_raw}
  \item Sett QoS \emph{reliability} til ``Best effort''
\end{enumerate}

\section*{Kamerakalibrering}
\subsection*{Forbered terminal}
\begin{lstlisting}[language=bash]
export QT_QPA_PLATFORM=xcb
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
\end{lstlisting}

\subsection*{Start kalibrering}
\begin{lstlisting}[language=bash]
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.024 \
  --ros-args \
    -r image:=/image_raw \
    -r camera:=/camera
\end{lstlisting}

\subsection*{I GUI}
\begin{itemize}
  \item Velg \textbf{Checkerboard}-modus
  \item Trykk \texttt{b} gjentatte ganger for å samle $\ge25$ prøver
  \item Klikk ``Calibrate''
  \item Du skal se:
\end{itemize}

\begin{lstlisting}
Wrote calibration data to /tmp/calibrationdata.tar.gz
\end{lstlisting}

\section*{Pakk ut YAML-fil}
\begin{lstlisting}[language=bash]
mkdir -p ~/.ros/camera_info
tar -xvf /tmp/calibrationdata.tar.gz \
    --strip-components=1 \
    -C ~/.ros/camera_info
\end{lstlisting}

\section*{Verifisere kalibrering}
\begin{lstlisting}[language=bash]
ros2 topic echo /camera/camera_info --once
\end{lstlisting}

Du skal nå se de kalibrerte $K$, $D$, $R$ og $P$-matrisene i meldingen.

\end{document}
