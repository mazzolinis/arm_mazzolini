Work in progress della tesi, per tenere traccia delle modifiche.

Copiare la repository all'interno del proprio workspace/src.
Per funzionare il codice ha bisogno di realsense ros, si può ottenere qui:
https://github.com/realsenseai/realsense-ros.git

Per braccio reale:
`colcon build --symlink-install && source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch arm_mazzolini weeder.launch.py use_sim_time:=false`
Questo comando avvia in automatico anche il pacchetto Realsense per la D455. 

Controllare in un altro terminale che l'albero di tf sia unico e non spezzato in 3:
`ros2 run tf2_tool view_frames`

Se non dovesse comunicare con i motori provare:
`ros2 launch arm_mazzolini test_real_arm.launch.py`
In questo modo i bracci oscilleranno attorno alla posizione zero

ATTENZIONE: i file "arm_mazzolini.xacro" e "arm_mazzolini_controller.yaml" sono da copiare su raspberry, gli id sono settati sull'ultima giornata.
Il file weeder_parameters.yaml contiene i nomi dei topic su cui pubblica la Realsense, erano stati impostati all'ultimo test, controllare che corrispondano.
