This is the project for sensing and alarming posture state in front of computer.

For Hardware,
An atmega128 module is connected to PC. And IMU sensor, Ultrasonic sensor, flex sensor, buzzer, LED are connected to the atmega128 module.
IMU sensor, Ultrasonic sensor, flex sensor are for sensing neck angle, distance between head and monitor, waist angle respectively.
buzzer and LED are for alarming real-time posture state.

Pose_Corrector.c will be detecting real-time data from sensors, and alarming current posture state with buzzer sound and LED light.

Pose_Corrector.hex and Pose_Corrector.elf are needed to write Pose_Corrector.c to the atmega128 module.
