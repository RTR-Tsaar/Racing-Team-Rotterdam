# Steer-by-wire

## Overzicht
Dit project bevat de complete software voor het steer-by-wire sub systeem voor de RU-25 van de Racing Team Rotterdam. In deze README wordt de complete code en hardware van dit project uitgelegd.

## Inhoudsopgave

-   [Kenmerken](#kenmerken)
-   [Aan de slag](#aan-de-slag)
-   [Code Uitleg](#code-uitleg)
	- [Map Structuur STM projecten](#map-structuur-stm-projecten)
	- [main](#main)
	- [motorControl](#motorcontrol)
	- [CANBus](#canbus)
	- [encoder](#encoder)
	- [currentSensor](#currentsensor)
-   [Hardware](#hardware)
	- [Breadboard aansluiting](#breadboard-aansluiting)
	- [PCB aansluiting](#pcb-aansluiting)
-   [Troubleshooting](#troubleshooting)


## Kenmerken
-   Bevat een kalibratieproces om na elke start-up de stuurhoek nauwkeurig te meten.
-   De stuuractuator wordt aangestuurd door een aanpasbare PID-regelaar, wat zorgt voor een hoge nauwkeurigheid.
-   Het systeem communiceert via de CAN-bus met andere drive-by-wire componenten.
## Aan de slag

## Code Uitleg
### Map structuur STM projecten
De belangrijkste mappen in een stm project zijn de `src` en `inc` mappen. Deze zijn te vinden in de `core` map. In de `src` map worden de alle `.cpp` files geplaatst. Dit zijn de files waar de classes en de logica van de code zit.
In de `inc` map zitten de header of `.hpp` files. Deze files worden gebruikt voor het initialiseren van de klasses en waardes in de klasses van de bijbehorende `.cpp` file.

---
### Uitleg werking code


De code van het **Steer-by-Wire** systeem is opgebouwd uit de volgende klassen: **Main, MotorControl, CANBus, Encoder** en **CurrentSensor**. Elke klasse heeft zijn eigen verantwoordelijkheden en bijbehorende functies.



### `main`

Zoals de naam al aangeeft, is dit de hoofdklasse van de applicatie. Deze fungeert als het startpunt van het programma en coördineert de interactie tussen de verschillende klassen.

#### **Belangrijke functies in de `Main` klasse**

##### **`HAL_CAN_RxFifo0MsgPendingCallback()`**

Deze functie wordt aangeroepen wanneer een CAN-busbericht binnenkomt. Op basis van de CAN-ID worden de ontvangen waarden uitgelezen en opgeslagen in variabelen.

##### **`check_CAN_errors()`**

Deze functie controleert of er CAN-busfouten zijn ontvangen en helpt bij het diagnosticeren van communicatieproblemen.

##### **`int main(void)`**

Dit is de hoofdfunctie van de `Main`-klasse en daarmee van de gehele applicatie.

-   Aan het begin van deze functie worden alle hardwarecomponenten geïnitialiseerd door het aanroepen van functies zoals:
    -   `HAL_Init()` (initialisatie van de HAL-bibliotheek)
    -   `SystemClock_Config()` (configuratie van de systeemklok)
    -  `MX_GPIO_Init()` (initialisatie van de GPIO)
    -   `MX_CAN_Init()` (initialisatie van de CAN-bus)
    -  `MX_TIM2_Init()` (initialisatie van de timer 2)
    - `MX_ADC1_Init()` (initialisatie van de analoog naar digitaal converter)
    - `MX_TIM3_Init()` (initialisatie van de timer 3)
-   Vervolgens draait de `while(1)`-loop, waarin het volledige programma continu blijft draaien.

##### **`SystemClock_Config()` en `..._Init` functies**

-   De functie `SystemClock_Config()` bepaalt de configuratie van de systeemklok van de STM32-microcontroller. Hoewel deze handmatig aangepast kan worden, wordt aangeraden om dit via de **`.ioc`-file** te doen. Dit maakt de configuratie van systeeminstellingen en communicatieprotocollen eenvoudiger.
-   Functies die eindigen op `_Init` zijn verantwoordelijk voor de configuratie van verschillende communicatieprotocollen (zoals CAN, UART, SPI, I2C). Net als bij `SystemClock_Config()` kunnen deze handmatig worden aangepast of via de `.ioc`-file.

#### **Let op: gebruik van de `.ioc`-file in combinatie met C++**

Wanneer je de **STM32CubeMX `.ioc`-file** gebruikt om de hardwareconfiguratie aan te passen, wordt de code automatisch gegenereerd in **C**. Als je met **C++** werkt, moet je een aantal aanpassingen doen:

1.  **Hernoem de bestanden** `main.c` en `main.h` naar `main.cpp` en `main.hpp`.
2.  **Kopieer je zelfgeschreven code** vanuit de oude `main.cpp` naar de nieuw gegenereerde `main.cpp` om verlies van aanpassingen te voorkomen.
3.  **Compatibiliteit behouden**: Omdat de HAL-bibliotheek in **C** is geschreven, moet je mogelijk `extern "C"` gebruiken in je `main.hpp` om de C-functies correct in te laden.




### `motorControl`

De `motorControl`-klasse is verantwoordelijk voor het aansturen van de stuuractuator. Dit omvat zowel het bepalen van de draairichting als de snelheid van de actuator.

#### `motorControl.hpp`

In het headerbestand van de `motorControl`-klasse worden alle functies en enkele variabelen gedeclareerd. Hier worden ook de parameters van de PID-regelaar gedefinieerd. Door deze waarden aan te passen, kunnen de karakteristieken van de PID-regelaar worden gewijzigd.

#### Belangrijke functies in de `motorControl`-klasse

##### `MotorControl()`

Dit is de constructor van de `motorControl`-klasse. De constructor wordt automatisch aangeroepen bij het aanmaken van een nieuw `motorControl`-object. Bij het aanmaken van dit object moeten een timer en een kanaal worden meegegeven, die worden gebruikt voor het genereren van het PWM-signaal voor de motordriver.

##### `start()`

Deze functie start het juiste PWM-kanaal voor de motordriver.

##### `setDutyCycle(uint16_t dutyCycle)`

Met deze functie kan handmatig de snelheid van de actuator worden geregeld. Dit gebeurt door als argument een duty cycle mee te geven met een waarde tussen 0 en 65.535. Hoe hoger de duty cycle, hoe sneller de motor draait.

##### `toggleDirection()`

Met deze functie kan de draairichting van de motor worden omgeschakeld.

##### `calculatePID(int currentAngle, int targetAngle)`

Deze functie implementeert de PID-regelaar die de draaisnelheid en -richting van de actuator bepaalt. Dit gebeurt op basis van de huidige stuurhoek (`currentAngle`) en de gewenste stuurhoek (`targetAngle`). De PID-regelaar berekent de fout en genereert een outputsignaal dat wordt gebruikt om de actuator aan te sturen. De prestaties van de PID-regelaar kunnen worden aangepast door de `KP`, `KI` en `KD`-waarden in het bestand `motorControl.hpp` te wijzigen.

##### `steerToAngle(int currentAngle, int targetAngle)`

De `steerToAngle()`-functie gebruikt de output van de `calculatePID()`-functie om een duty cycle en draairichting te bepalen. Deze waarden worden vervolgens doorgegeven aan de motordriver om de actuator aan te sturen.

### `CANBus`

De `CANBus` klasse is verantwoordelijk voor het behandelen van het sturen en ontvangen van berichten over de CAN-bus lijn. 

#### Belangrijke functies in de `CANBus`-klasse

##### `start()`
Deze functie start het CAN-busprotocol en activeert de CAN-notificaties, zodat CAN-busberichten via interrupts kunnen worden ontvangen. Daarnaast bereidt deze functie de header voor het verzenden van een CAN-busbericht voor.
##### `transmit()`
Deze functie wordt gebruikt voor het verzenden van CAN-busberichten. Om deze functie te gebruiken, moeten een CAN-handle, een dataframe en een CAN-ID worden meegegeven.
##### `configureFilter()`
In `configureFilter()` worden alle filter intellingen bepaald. Als je bijvoorbeeld de filterId of filterMask wil aanpassen doe je dat hier.
##### `error()`
Deze functie kan gebruikt worden om een error bericht over de CAN lijn te sturen. Hierbei is de verstuurde data NULL.
##### `storeCAN()`
De functie `storeCAN()` is bedoelt om alle via CAN-bus ontvange data op te slaan in een stack, om er later weer bij te kunnen.
##### `dataSplitter()`
De `dataSplitter()` functie maakt het mogelijk om waardes groter dan een byte via CAN-bus te sturen. Dit doet het door de waardes in een lijst van enkele bytes te splitten die dan als dataframe verstuurd kan worden.
##### `dataMerger()`
`dataMerger()` doet het omgekeerde van `dataSplitter()`. Bij ontvangst van een bericht voegt het de bytes uit de dataframe weer samen om een grote waarde te vormen.
##### `getLastData()`
`getLastData()` haalt de laatst opgeslagen waarde op uit de `storeCAN()` stack.

### `encoder`

De encoder klasse is verntwoordelijk voor het kalibreren en uitlezen van de encoders om zo de stuurhoek van de RU-25 te bepalen.

#### Belangrijke functies in de `encoder`-klasse


### `currentSensor`
De `currentSensor` klasse is verantwoordelijk voor het uitlezen van de stroomsensoor dat gebruikt wordt bij het kalibratie proces van de encoders.

#### Belangrijke functies in de `currentSensor`-klasse
##### `CurrentSensor()`
Dit is de constructor voor de `CurrentSensor` klasse. Deze functie wordt aangeroepen bij het maken van een `CurrentSensor` object. Hierbij moet de gebruikte adc worden megegeven, samen met de referentie spanning, de gevoeligheid en de offset voor de stroomsensoor. 

- `vRef`:
De referentie spanning, `vRef` is gelijk aan de spanning die gebruikt wordt voor het aansturen van de stroomsensoor op de `VCC` pin. In het geval van het steer by wire systeem is dit 3.3 volt.

- `sensitivity`:
De `sensitivity` bepaald hoeveel mV gelijk is aan 1 Ampere. Voor de 5 ampere variant van de `acs712` stroomsensoor is dit `185mV`. De 20 ampere variant heeft een gevoeligheid van `150mV`. Als er een andere stroomsensoor gebruikt wordt kan de gevoeligheid in de bijbehorend datasheet gevonden worden.

- `offset`:
De offset is de standaart voltage die de stroomsensoor als output geeft als er 0 ampere doorheen graat. De offset kan berekend wordden door de `vRef` door 2 te delen. In dit geval is dit `1.65`. 
**Let op:** Vaak moet de offset nog handmatig gekalibreert worden om geheel acuraat te zijn.

##### `start()`en `stop()`
- De `start()` functie start de adc kanaal die gebruikt wordt door de stroomsensoor.
- De `stop()` functie zet de adc weer uit als die niet meer nodig is om zo resources te besparen.

##### `getCurrent()`
De `getCurrent()` functie leest de analoge output van de stroom sensoor met `HAL_ADC_GetValue()`. Hierna wordt deze rauw waarde omgezet naar amperen met de volgende berekening `Amperage = offset - (adcValue x (vRef/4095))/sensitivity`. Hierbij is `adcValue` de analoge output van de stroomsensoor.



## Hardware

Dit subsysteem kan zowel op een **breadboard** als op de **custom PCB** worden uitgevoerd. Hieronder volgt een stappenplan voor beide aansluitmethoden.
### Breadboard aansluiting
Om dit subsysteem te gebruiken op een breadboard zijn de volgende onderdelen nodig:
- 1x STM32C6T6a microcontroller
- 1x ST-Link v2 connector
- 1x SN65HVD230 CAN Tranceiver Module
- 2x 5k ohm weerstand
- 2x 9.84k ohm weerstand
- 1x acs712 stroomsensoor
- 1x Nidec RE30E-1000-213-1 encoder
- 1x MD20A of MD13S motordriver
- 1x DC motor

#### Aansluiting
Met de bovenstaande onderdelen kan het systeem als volgt worden aangesloten:
![breadboard aansluiting steer by wire](Docs/Steer-by-wire%20breadboard%20aansluiting.png)


### PCB aansluiting
Om dit systeem te gebruiken op de submodule PCB zijn de volgende onderdelen nodig:
- 1x STM32C6T6a microcontroller
- 1x ST-Link v2 connector
- 1x Nidec RE30E-1000-213-1 encoder
- 1x MD20A of MD13S motordriver
- 1x DC motor
- 1x submodule PCB
#### Aansluiting
![pcb aansluiting steer by wire](Docs/Steer-by-wire%20pcb%20aansluiting.jpeg)

Voor meer informatie over de sub module pcb, download het [KiCad project](https://github.com/RTR-Tsaar/Racing-Team-Racing-Team-Rotterdam/Docs/RTR%20submodule/RTR_submodule) en open het in kicad.
[KiCad Download Link](https://www.kicad.org/download/)

> Written with [StackEdit](https://stackedit.io/).
