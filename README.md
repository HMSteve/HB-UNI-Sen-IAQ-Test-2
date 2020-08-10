
# Zweites Test-Device fuer Indoor Air Quality Sensor BME680

Vielen Dank an Jerome (jp112sdl) und TomMajor, auf deren Arbeit dieses Projekt weitgehend aufbaut!

Ziel ist ein Device, das neben den Messwerten des BME680 Referenzmesswerte alternativer Sensoren ermittelt, um das Verhalten des BME680 zu vergleichen.

Aenderung gegenueber der ersten Version (HB-UNI-Sen-IAQ-Test) ist die Nutzung des SGPC3 statt des SGP30, um Batteriebetrieb zu ermoeglichen. Das AddOn ist identisch, der Datenpunkt eCO2 wird nicht genutzt.

- Hardware basierend auf ATMega1248p wg dubioser Speicherplatzprobleme beim 328p
- Es wird neben dem BME680 ein SHT31 fuer Temperatur und Feuchte, ein BMP280 fuer Luftdruck und ein SGPC3 fuer VOC genutzt.
- Das rudimentaere Addon stellt die Werte aller Sensoren in der CCU bereit, um hier existierendes Logging fuer die Zeitreihengewinnung zwecks anschliessenden Vergleiches der Sensorverhalten nutzen zu koennen.

Da SGPC3 Breakout Boards im Standardformat mit I2C-Schnittstelle nicht am Markt zu finden waren, habe ich ein eigenes erstellt: [SGPC3 Breakout Board](https://github.com/HMSteve/PCBs/tree/master/SGPC3_BreakoutBoard).



## Lizenz

**Creative Commons BY-NC-SA**<br>
Give Credit, NonCommercial, ShareAlike

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.
