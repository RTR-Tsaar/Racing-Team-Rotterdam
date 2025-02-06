
# Steer-by-wire

## Overzicht
Dit project bevat de complete software voor het throttle-by-wire sub systeem voor de RU-25 van de Racing Team Rotterdam. In deze README wordt de complete code en hardware van dit project uitgelegd.

## Inhoudsopgave

-   [Kenmerken](#kenmerken)
-   [Aan de slag](#aan-de-slag)
-   [Code Uitleg](#code-uitleg)
	- [Map Structuur STM projecten](#map-structuur-stm-projecten)
	- [main](#main)
	- [CANBus](#canbus)
	- [Throttle](#throttle)
-   [Hardware](#hardware)
	- [Breadboard aansluiting](#breadboard-aansluiting)
	- [PCB aansluiting](#pcb-aansluiting)


## Kenmerken
- Gaspedaal uitlezen: Leest de positie van een elektronisch gaspedaal via een analoge ingang (ADC).
- CAN-bus communicatie: Verstuurt de gaspedaalpositie als een percentage (0-100%) over CAN-bus.
- Lage latentie: Geoptimaliseerde firmware voor snelle respons en minimale vertraging tussen input en output.
- STM32-gebaseerd: Gebouwd op een STM32-microcontroller met HAL- of LL-bibliotheken.
## Aan de slag

## Code Uitleg
### File structuur STM projecten
De belangrijkste mappen in een stm project zijn de `src` en `inc` mappen. Deze zijn te vinden in de `core` map. In de `src` map worden de alle `.cpp` files geplaatst. Dit zijn de files waar de classes en de logica van de code zit.
In de `inc` map zitten de header of `.hpp` files. Deze files worden gebruikt voor het initialiseren van de klasses en waardes in de klasses van de bijbehorende `.cpp` file.

### Uitleg werking code

---

De code van het **Throttle-by-Wire** systeem is opgebouwd uit de volgende klassen: **Main, Throttle** en **CANBus** . Elke klasse heeft zijn eigen verantwoordelijkheden en bijbehorende functies.

### ` Main`

Zoals de naam al aangeeft, is dit de hoofdklasse van de applicatie. Deze fungeert als het startpunt van het programma en coördineert de interactie tussen de verschillende klassen.

#### Belangrijke functies in de `Main` klasse

##### `HAL_CAN_RxFifo0MsgPendingCallback()`

Deze functie wordt aangeroepen wanneer een CAN-busbericht binnenkomt. Op basis van de CAN-ID worden de ontvangen waarden uitgelezen en opgeslagen in variabelen.

##### `check_CAN_errors()`

Deze functie controleert of er CAN-busfouten zijn ontvangen en helpt bij het diagnosticeren van communicatieproblemen.

##### `int main(void)`

Dit is de hoofdfunctie van de `Main`-klasse en daarmee van de gehele applicatie.

-   Aan het begin van deze functie worden alle hardwarecomponenten geïnitialiseerd door het aanroepen van functies zoals:
    -   `HAL_Init()` (initialisatie van de HAL-bibliotheek)
    -   `SystemClock_Config()` (configuratie van de systeemklok)
    -  `MX_GPIO_Init()` (initialisatie van de GPIO)
    -   `MX_CAN_Init()` (initialisatie van de CAN-bus)
    - `MX_ADC1_Init()` (initialisatie van de analoog naar digitaal converter)
-   Vervolgens draait de `while(1)`-loop, waarin het volledige programma continu blijft draaien.

##### **`SystemClock_Config()` en `..._Init` functies**

-   De functie `SystemClock_Config()` bepaalt de configuratie van de systeemklok van de STM32-microcontroller. Hoewel deze handmatig aangepast kan worden, wordt aangeraden om dit via de **`.ioc`-file** te doen. Dit maakt de configuratie van systeeminstellingen en communicatieprotocollen eenvoudiger.
-   Functies die eindigen op `_Init` zijn verantwoordelijk voor de configuratie van verschillende communicatieprotocollen (zoals CAN, UART, SPI, I2C). Net als bij `SystemClock_Config()` kunnen deze handmatig worden aangepast of via de `.ioc`-file.

#### **Let op: gebruik van de `.ioc`-file in combinatie met C++**

Wanneer je de **STM32CubeMX `.ioc`-file** gebruikt om de hardwareconfiguratie aan te passen, wordt de code automatisch gegenereerd in **C**. Als je met **C++** werkt, moet je een aantal aanpassingen doen:

1.  **Hernoem de bestanden** `main.c` en `main.h` naar `main.cpp` en `main.hpp`.
2.  **Kopieer je zelfgeschreven code** vanuit de oude `main.cpp` naar de nieuw gegenereerde `main.cpp` om verlies van aanpassingen te voorkomen.
3.  **Compatibiliteit behouden**: Omdat de HAL-bibliotheek in **C** is geschreven, moet je mogelijk `extern "C"` gebruiken in je `main.hpp` om de C-functies correct in te laden.


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

#### `Throttle`
De `Throttle`-klasse is verantwoordelijk voor het uitlezen van het gaspedaal en het converteren van de ruwe data naar een procentuele waarde.
#### Belangrijke functies in de `Throttle`-klasse
##### `readThrottle()`
De `readThrottle()` functie is verantwoordelijk voor het uitlezen van de juiste adc kanaal voor het gaspedaal. Het retourneert de gaspedaal input als percentage die makkelijk via CAN-bus verstuurd kan worden.

## Hardware
Dit subsysteem kan zowel op een **breadboard** als op de **custom PCB** worden uitgevoerd. Hieronder volgt een stappenplan voor beide aansluitmethoden.
### Breadboard aansluiting
Om dit subsysteem te gebruiken op een breadboard zijn de volgende onderdelen nodig:
- 1x STM32C6T6a microcontroller
- 1x ST-Link v2 connector
- 1x 0-5v analoog gaspedaal
- 1x SN65HVD230 CAN Tranceiver Module
- 1x 5k ohm weerstand
- 1x 9.84k ohm weerstand

#### Aansluiting
Met de bovenstaande onderdelen kan het systeem als volgt worden aangesloten:
![breadboard aansluiting throttle by wire](Docs/throttle-by-wire%20breadboard%20aansluiting.png)

### PCB aansluiting
Om het systeem aan te sluiten via de sub module pcb zijn de volgende onderdelen:
- 1x STM32C6T6a microcontroller
- 1x ST-Link v2 connector
- 1x 0-5v analoog gaspedaal
- 1x submodule PCB
#### Aansluiting
Deze onderdelen kunnen daarna als volg worden aangesloten aan de pcb:
![pcb aansluiting throttle by wire](Docs/Throttle%20pcb%20wiring.jpeg)

Voor meer informatie over de sub module pcb, download het [KiCad project](https://github.com/RTR-Tsaar/Racing-Team-Rotterdam/tree/main/Docs/RTR%20submodule) en open het in kicad.
[KiCad Download Link](https://www.kicad.org/download/)
Voor meer informatie over de pcb onderdelen ga naar [Racing-Team-Rotterdam/Docs/Sub module components](https://github.com/RTR-Tsaar/Racing-Team-Rotterdam/tree/main/Docs/Sub%20module%20components)


> Written with [StackEdit](https://stackedit.io/).
