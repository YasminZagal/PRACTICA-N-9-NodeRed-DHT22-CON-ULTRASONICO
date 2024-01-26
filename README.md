# PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO

Este repositorio se utilizara el flow anterioragregandole un nodo mas, en la cual se utilizo NodeRed y WOKWI

## Introducción

El flow 2 representa el segundo ejercicio a realizar con NodeRed. Este ejercicio consiste únicamente en hacer conexion a un servidor publico.

### Descripción

La ```Esp32``` la utilizamos en un entorno de adquision de datos, lo cual en esta practica ocuparemos un sensor (```DTH22```) para la obtención de datos de temperatura y humedad, ademas de un sensor (```Ultrasonico HC-SR04```)  medicion de distancia; Esta practica se usara un simulador llamado [WOKWI](https://wokwi.com/), junto con NodeRed.


## Material Necesario

Para realizar este flow necesitas lo siguiente

- [Node.js](hhttps://nodejs.org/en)
- [WOKWI](https://https://wokwi.com/)
- Tarjeta ESP 32
- Sensor DHT22
- Sensor ultrasonico HC-SR04
  


## Instrucciones

### Requisitos previos

Para que este flow funcione, debes cumplir con los siguientes requisitos previos
1. Para realizar la practica de este repositorio se necesita entrar a la plataforma [WOKWI](https://https://wokwi.com/).
2. Tener instalado Node.js (version 20.11.0 LTS).

### Instrucciones de preparación del entorno en wokwi

1. Abrir la terminal de programación y colocar la siguente codigo:

```
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2
#include "DHTesp.h"
const int DHT_PIN = 15;
const int Trigger = 4;   //Pin digital 2 para el Trigger del sensor
const int Echo = 2;   //Pin digital 3 para el Echo del sensor
DHTesp dhtSensor;
// Update these with values suitable for your network.

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "18.193.219.109";
String username_mqtt="educatronicosiot";
String password_mqtt="12345678";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  
    // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

void loop() {


delay(1000);
TempAndHumidity  data = dhtSensor.getTempAndHumidity();
long t; //timepo que demora en llegar el eco
long d; //distancia en centimetros

digitalWrite(Trigger, HIGH);
delayMicroseconds(10);          //Enviamos un pulso de 10us
digitalWrite(Trigger, LOW);
  
t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
d = t/59;             //escalamos el tiempo a una distancia en cm
  
Serial.print("Distancia: ");
Serial.print(d);      //Enviamos serialmente el valor de la distancia
Serial.print("cm");
Serial.println();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    //++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);

    StaticJsonDocument<128> doc;

    doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    doc["DISTANCIA"] = String(d);
    doc["TEMPERATURA"] = String(data.temperature, 1);
    doc["HUMEDAD"] = String(data.humidity, 1);
   

    String output;
    
    serializeJson(doc, output);

    Serial.print("Publish message: ");
    Serial.println(output);
    Serial.println(output.c_str());
    client.publish("yasminzagal", output.c_str());
  }
}
```

2. Instalar la libreria de **ArduinoJson** **DTH sensor library for ESPx** **PubSubClient**. 
   - Seleccionar pestaña de Librery Manager --> Add a New library --> Colocamos el nombre de libreria
 ![](https://github.com/YasminZagal/PRACTICA-N-8-NodeRed-CON-DHT22/blob/main/librerias.png)
  
3. Realizar la conexion de **DTH22** con la **ESP32** de la siguiente manera.
 ![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/conexciones.png)
     
  **Conexión DTH22**
  -VCC --> GND
  -SDA --> esp:15
  -NC 
  -GND  --> 5V

  **Conexión HC-SR04**
  -GND --> GND
  -ECHO --> esp: 2
  -TRIG --> esp: 4
  -VCC --> 5V

  

### Instrucciones de preparación del entorno en NodeRed

Para ejecutar este flow, es necesario lo siguiente
1. Arrancar el contenedor de NodeRed en **cmd** con el comando
        
        node-red

2. Dirigirse a [localhost:1880](localhost:1880)

3. En la parte izqierda de la pantalla seleccionaremos
   - 1 mqtt in
   - json
   - 3 fuction
   - 3 gauge
   - 3 chart

 ![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/nodos.png)
  
4. Para la configuracion del mqtt in necesitaremos saber nuestra ip, que se saca de la siguiente manera:
   cmd --> nslookup broker.hivemq.com --> copiamos los numeros de la parte que dice addresses --> nos dirigimos a nodered --> seleccionamos mqtt in --> damos click en el icono del lapiz --> en la parte de server se pegara la direccion ip que copiamos --> update --> done

![](https://github.com/YasminZagal/PRACTICA-N-8-NodeRed-CON-DHT22/blob/main/direccion%20ip.png)   

![](https://github.com/YasminZagal/PRACTICA-N-8-NodeRed-CON-DHT22/blob/main/conf1.png)  

5. En la configuracion del **JSON** lo seleccionamos y en la parte de **action** se despelgaran unas opciones en donde colocaremos **always convert JavaScript Object**

![](https://github.com/YasminZagal/PRACTICA-N-8-NodeRed-CON-DHT22/blob/main/conf2.png)

6. En el primer function se colocara el mobre de **TEMPERATURA** y posteriormente se pondra el siguiente codigo

```
msg.payload = msg.payload.TEMPERATURA;
msg.topic = "TEMPERATURA";
return msg;
```
![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/conf%20nodo%20temp.png)

7.En el segundo fuction se llamara **HUMEDAD** e igualmente se colocara un codigo

```
msg.payload = msg.payload.HUMEDAD;
msg.topic = "HUMEDAD";
return msg;
```
![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/conf%20nodo%20hum.png)

8. En el terecer fuction se llamara **DISTANCIA** con s codigo correspondiente 

```
msg.payload = msg.payload.DISTANCIA;
msg.topic = "DISTANCIA";
return msg;
```
![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/conf%20nodo%20dis.png)



9. Y por ultimo en la parte dashboard se agrega una nueva tabla con el mobre de sensor DHT, posteriormente se añaden dos grupos uno con el nombre de INDICADORES y el otro con GRAFICAS. En el nodo de gauge de los tres functions se colocara en el grupo de sensor dht indicadores y en el chart de los  funcion sus nodos corresponderan al grupo de sensor dth graficas

![](https://github.com/YasminZagal/PRACTICA-N-8-NodeRed-CON-DHT22/blob/main/conf%20grafica%20hum.png)
![](https://github.com/YasminZagal/PRACTICA-N-8-NodeRed-CON-DHT22/blob/main/conf%20grafica%20temp.png)

   
### Instrucciones de operación
1. Nos vamos a nuestro wokwi en donde nuestro simulador ya esta corriendo
![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/resultado%201.3.png)

## Resultados

A continuación se puede observar una vista previa del resultado del flow2 y la interaccion con el wokwi .

![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/resultado%201.1.png)
![](https://github.com/YasminZagal/PRACTICA-N-9-NodeRed-DHT22-CON-ULTRASONICO/blob/main/resultado%201.2.png)



