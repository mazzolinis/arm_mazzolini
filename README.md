Work in progress della tesi, per tenere traccia delle modifiche.

## Setup

Copiare la repository all'interno del proprio workspace/src.
Per funzionare il codice ha bisogno di realsense ros, si può ottenere qui:
[realsense-ros](https://github.com/realsenseai/realsense-ros.git)

## Avvio moduli da raspberry

```bash
sudo su
source install/setup.bash
export ROS_DOMAIN_ID=21 
ros2 launch pi3hat_hw_interface moteus_pi3hat_interface.launch.py urdf_file:=arm_mazzolini.xacro conf_file:=arm_mazzolini_controller.yaml
```
su nuovo terminale
```bash
ros2 service call /omni_controller/activate_srv std_srvs/srv/SetBool "{data: true}"  
```

## Avvio braccio reale

```bash
colcon build --symlink-install && source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch arm_mazzolini weeder.launch.py use_sim_time:=false
```

Questo comando avvia in automatico anche il pacchetto Realsense per la D455.

Controllare in un altro terminale che l'albero di tf sia unico e non spezzato in 3:

```bash
ros2 run tf2_tool view_frames
```

Se non dovesse comunicare con i motori provare:

```bash
ros2 launch arm_mazzolini test_real_arm.launch.py
```

In questo modo i bracci oscilleranno attorno alla posizione zero.

## Attenzione

- I file `arm_mazzolini.xacro` e `arm_mazzolini_controller.yaml` sono da copiare su raspberry; gli ID sono settati sull'ultima giornata.
- Il file `weeder_parameters.yaml` contiene i nomi dei topic su cui pubblica la Realsense; erano stati impostati all'ultimo test, controllare che corrispondano.

## Utilizzo segmentation con YOLOv11

Impostando `use_sim_time:=true` si attiva la rete YOLO. 
Verificare che si stia utilizzando una scheda video e che le impostazioni CUDA siano corrette.
