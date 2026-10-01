# Common Mistakes And Definitions

> 出典: https://support.unitree.com/home/en/G1_developer/common_istakes_and_definitions

Source of failure	Error code	Wrong definition	
Total device status information	0x1	Upper control command error	
0x2	Lower layer feedback data timeout	
0x4	imu feedback data timeout	
0x8	Motor feedback data timeout	
0x10	Battery feedback data timeout	
0x20	Yushu entity remote control feedback data timeout	
0x40	Battery model error	
0x80	Soft-start error	
0x100	Motor state error	
0x200	motor overcurrent protection, trigger low limit protection.	
0x400	Motor undervoltage protection, trigger high limit protection	
0x800	Motor overcurrent protection, trigger high limit protection	
0x1000	Soft emergency stop switch is pressed	
0x2000	SN error	
0x4000	Upper model error	
0x8000	Lower model error	
0x10000	USB device error	
0x40000	Crotch IMU data timeout	
0x80000	Motherboard judgment battery undervoltage protection error	
0x100000	Motherboard judge motor undervoltage protection error	
Motor status information	0x1	Overcurrent	
0x2	Transient overvoltage	
0x4	Sustained overpressure	
0x8	Transient undervoltage
0x10	Chip overheating	
0x20	MOS overheating/cooling	
0x40	MOS temperature anomaly	
0x80	Shell overheat/undercooling	
0x100	Shell temperature anomaly	
0x200	Winding overheating	
0x400	Rotor encoder 1 error	
0x800	Rotor encoder 2 error	
0x1000	Output encoder error	
0x2000	Calibration /BOOT data error	
0x4000	Abnormal reset	
0x800	Rotor encoder 2 error	
0x1000	Output encoder error	
0x2000	Calibration /BOOT data error	
0x4000	Abnormal reset	
0x8000	Motor locked, master authentication error	
0x10000	Chip validation error	
0x20000	Calibration mode warning	
0x40000	Communication verification error	
0x80000	driver version too low	
0x40000000	Motor determines that PC connection timeout	
0x80000000	PC interpretation, motor disconnection timeout	