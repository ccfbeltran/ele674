# ElE674 Laboratoire 3 Firmware
## Demarrer le projet sur Eclipse

1. Aller sur abidos

	`share/sa/Lab/Lab3-firmware/DroneFirmwareBeaglebone` (code a completer)

2. Configuration de la connexion a distance de la plateforme cible (dans le tutoriel de eclipse page 12) (Savoir comment Debug)

3. Copier et coller le projet DroneFirmwareBeaglebone

4. Demarrer eclipse et Importer le projet

5. Voir si ya des path a changer dans properties->Settings


## Les logs

exemple: ./DroneFirmware LogAll



## Boot

Booter le Drone avec lapplication Terminale du port serie

Ouvrir un autre terminal 
```shell
	$ ifconfig -a
	
	regarder pour usb_beaglebone
	$ sudo ifconfig -s usb_beaglebone 192.168.5.1 
	$ ifconfig
	regarder le nouveau ip address
	pour tester $ ping 192.168.5.200

	$ ssh root@192.168.5.200 (connection sur le drone)
	sinon
	sftp://root@192.168.5.200 sur le dossier

	Dans DroneFirmwareBeaglebone sur Debug pendre DroneFirmwareBeaglebone et i2c_sensors.ko
	mettre sur le root du beaglebone
	
	$ insmod i2c_sensors.ko 
	$ ls -la /dev/i2c* (regarder les pilotes installer)
```
## QGroundcontrol

Si le ip adresse bug
```shell
./DroneFirmwareBeagleBone IP=192.168.5.1
```
ou IP=192.168.5.5 (a verifier)

Pour voir les plot

`Main Widget-->Realtimeplot`


Pour controller la vitesse des moteurs avec Qgroundcontrole

`File-->ControlWidget-->Activate control`
