# SUTS-iacb
Arendusplaadil ja maketil B-doti arendus
NB:repost puudub praegu Debug kaust(liiga suur), peaks olema jooksutatav ka ilma?
Core:Inc Src ja muud põhilised failid
Drivers:Magnetomeetri driveritega kaust, mis sisaldab veel teiste sensorite drivereid

Uuenduslogi:
04/02 - I2C ühendus katki, ei saa pärida LIS3MDL-ilt WHO_AM_I registrit ja sellest tulenevalt edasi ei lähe koodis. 
MCU poole pealt saab liides initsialiseeritud kuid magnetomeetrilt vastust ei tule. Draiverid implementeeritud ja sellega kaasa käivad funktsioonid juba koodis olemas, sensoriga ühenduse saamisel peaks olema üsnagi kerge vaev andmeid lugeda.

09/02 - SPI töötab, pole hetkel kindel kas ka nii nagu peaks kuid andmed tunduvad üsna korrektsed. Vajab testimist.
