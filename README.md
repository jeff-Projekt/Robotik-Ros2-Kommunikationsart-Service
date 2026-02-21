# Praktikum-Hochschule-Robotik
akademisches Praktikum-ROS2-Service / Ingenieurwesen / Labor
Hochschule : Technische Hochschule Georg Agricola
Wissenschaftsbereich : Elektrotechnik (BET-Energietechnik)
Team : 2 Studenten

Dieses Projekt implementiert eine Client-Server-Architektur in ROS2, mit der die Scanfrequenz eines simulierten Laserscanners dynamisch über einen benutzerdefinierten ROS2-Service geändert werden kann.
Das Projekt erweitert die vorherige Implementierung auf Basis von ROS2-Topics um einen Service-Mechanismus (Request/Response).
Ziel ist es, die Unterschiede zwischen kontinuierlicher Kommunikation (Topics) und ereignisgesteuerter Kommunikation (Services) praktisch zu verstehen und umzusetzen.

    Systemarchitektur
https://docs.ros.org/en/kilted/_images/Service-SingleServiceClient.gif
https://docs.ros.org/en/foxy/_images/Service-MultipleServiceClient.gif
https://docs.ros.org/en/rolling/_images/Nodes-TopicandService.gif

             Architekturüberblick

Das System besteht aus drei zentralen Komponenten:
Service-Interface Package
Service-Server (LaserPublisherNode)
Service-Client (SetScanFrequencyClientNode)

             Kommunikationsablauf

Der Client sendet eine Request mit einer gewünschten Scanfrequenz.
Der Server überprüft die Gültigkeit der Frequenz.

Falls gültig:
Der bestehende Timer wird gestoppt.
Ein neuer Timer mit der gewünschten Frequenz wird erzeugt.
Eine erfolgreiche Response wird zurückgesendet.

Falls ungültig:
Die Anfrage wird abgelehnt.
Eine entsprechende Fehlermeldung wird zurückgegeben.

    Projektstruktur
ros2_ws/
 ├── src/
 │   ├── v1_topic_pkg/
 │   │   ├── laser_data_publisher.py
 │   │   ├── set_scan_frequency_client.py
 │   │
 │   ├── v2_service_interface/
 │       ├── srv/
 │           ├── SetScanFrequency.srv


              Verwendete Technologien
ROS2
Python 3
ament_cmake
colcon build
Ubuntu Linux
Visual Studio Code

    Teil 1 – Eigenen ROS2-Service definieren

                Package: v2_service_interface
Der Service wird in einem separaten Interface-Package definiert.
Datei: SetScanFrequency.srv
int64 scan_frequency
---
bool success
string message

                 Bedeutung der Felder
Teil	Feld	Typ	Beschreibung
Request	scan_frequency	int64	Gewünschte Scanfrequenz in Hz
Response	success	bool	Gibt an, ob die Anfrage akzeptiert wurde
Response	message	string	Statusmeldung des Servers

    Teil 2 – Service-Server
Node: laser_data_publisher
Dieser Node simuliert einen Laserscanner und bietet den Service:
/set_scan_frequency

                  Zulässige Frequenzen
VALID_SCAN_FREQUENCIES = [5, 10, 15]

                  Funktionsweise
Empfang der Client-Anfrage
Validierung der Frequenz
Dynamische Anpassung des ROS2-Timers
Rückgabe einer strukturierten Response

                  Eigenschaften des simulierten Laserscanners
Parameter	Wert
Startwinkel	-90°
Endwinkel	+90°
Scanbereich	180°
Winkelauflösung	1°
Messbereich	0.15 m – 10.0 m
Frequenzen	5 / 10 / 15 Hz

    Teil 3 – Service-Client
Node: set_scan_frequency_client
Der Client sendet eine asynchrone Service-Anfrage und verarbeitet die Antwort mittels Callback-Funktion.

                  Ablauf

Warten auf Service-Verfügbarkeit
Erstellen einer Request-Nachricht
Asynchroner Service-Aufruf (call_async)
Verarbeitung der Serverantwort über future

                  Projekt ausführen
Workspace bauen
cd ~/ros2_ws
colcon build
source install/setup.bash

                  Service-Server starten
ros2 run v1_topic_pkg laser_data_publisher

                  Service-Client starten
ros2 run v1_topic_pkg set_scan_frequency_client

                  Manueller Test via Terminal
ros2 service call /set_scan_frequency v2_service_interface/srv/SetScanFrequency "{scan_frequency: 5}"

                   Lernziele & Erworbene Kompetenzen

                   Fachliche Kompetenzen
Verständnis der ROS2-Service-Architektur
Erstellung eigener .srv-Dateien
Implementierung von Service-Servern und -Clients
Asynchrone Kommunikation mit Futures
Dynamische Timer-Anpassung in ROS2

                   Programmiertechnische Kompetenzen
Strukturierung eines ROS2-Workspaces
Konfiguration von package.xml und CMakeLists.txt
Dependency-Management mit rosdep
Objektorientierte Python-Programmierung
Logging und Fehlerbehandlung in ROS2

                   Robotik-Kompetenzen
Simulation eines Laserscanners
Parametrisierung von Sensorfrequenzen
Verständnis ereignisgesteuerter Robotersysteme
Umsetzung einer modularen Systemarchitektur
