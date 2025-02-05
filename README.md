# Racing-Team-Rotterdam

## Overzicht

Deze repository bevat de software voor het drive-by-wire systeem, ontwikkeld voor de Racing Team Rotterdam Formula Student-auto. Het systeem is verdeeld over meerdere STM-microcontrollers, die elk verantwoordelijk zijn voor een specifieke subsystem. Samen zorgen ze voor precieze en betrouwbare controle van de stuur- en gaspedaalfunctionaliteit.

## Systeemcomponenten

Het drive-by-wire systeem is modulair opgebouwd, waarbij elke microcontroller een specifieke functie vervult:
1. **MCU**
	-	De MCU controleert alle subsystemen via CAN-bus. Het is de interface tussen de subsystemen en het toekomstige autonoom systeem.
	-	Is het brein van het complete systeem.
2.  **Steer-by-Wire**
    
    -   Verwerkt stuurinputs en bestuurt de elektronische stuuractuator.
    - Registreert huidige stuurhoek en geeft deze door aan de MCU.
3.  **Throttle-by-Wire**
    
    -   Registreert pedaalinputs en geeft deze door aan de MCU voor nauwkeurige motorbesturing.
4.  **Brake-by-wire**
    
    -   Registreert pedaalinputs en geeft deze door aan de MCU voor nauwkeurige motorbesturing.
5.  **Powertrain**
   
    -   Ontvangt gas en rem inputs van de MCU en stuurt hiermee de motorcontrollers aan.
    - Leest de CAN-bus berichten uit van de accu en geeft de belangrijkste door aan de MCU


## Inhoudsopgave

-   [Kenmerken](#kenmerken)
-   [Aan de slag](#aan-de-slag)
-   [Installatie](#installatie)
-   [Projectstructuur](#projectstructuur)
-   [Gebruik](#gebruik)
-   [Bijdragen](#bijdragen)
-   [Licentie](#licentie)
-   [Contact](#contact)

## Kenmerken

-   Modulair drive-by-wire systeem met gescheiden microcontrollers.
-   CAN bus communicatie voor naadloze integratie van subsystemen.
-   Hoge responsiviteit voor nauwkeurige controle in Formula Student-races.

## Aan de slag

Om het systeem lokaal op te zetten en te testen, volg je deze stappen:

### Via Github Desktop

1.  **Repository klonen**
    
    -   Open GitHub Desktop.
    -   Klik op **File** > **Clone Repository**.
    -   Ga naar het tabblad **URL** en voer de volgende link in:
        
        ```
        https://github.com/RTR-Tsaar/Racing-Team-Rotterdam.git
        
        ```
        
    -   Kies een lokale map waar je de repository wilt opslaan en klik op **Clone**.
2.  **Project openen**
    
    -   Navigeer naar de lokaal gekloonde map via Verkenner (Windows) of Finder (Mac).
    -   Open de map van het subsystem waaraan je wilt werken, bijvoorbeeld `steer-by-wire/`.
3.  **STM32CubeIDE installeren**
    
    -   Als je dat nog niet hebt gedaan, installeer [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
    -   Open de folder van het gekozen subsystem in STM32CubeIDE.


### Via Terminal
1.  Clone deze repository:
    
    ```bash
    git clone https://github.com/RTR-Tsaar/Racing-Team-Rotterdam.git
    cd Racing-Team-Rotterdam
    
    ```
    
2.  Installeer [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) of een andere ontwikkelomgeving voor STM-microcontrollers.
    
3.  Open de map van het subsystem dat je wilt gebruiken.
    

## Installatie

Elk subsystem bevindt zich in een aparte map:

-   `steer-by-wire/`
-   `throttle-by-wire/`
-   `brake-by-wire/`
-   `aandrijving/`
-   `mcu/`
-   `CAN_bus_algemeen` (CAN-bus poc en testomgeving)

Volg de instructies in de README-bestanden van de respectieve mappen om de software te bouwen en te uploaden naar de bijbehorende microcontroller. Hier vind je ook meer uitleg over de werking van de code.

## Projectstructuur

```
Racing-Team-Rotterdam/
├── CAN_bus_algemeen/ (CAN-bus poc en testomgeving)
├── steer-by-wire/  
├── throttle-by-wire/  
├── brake-by-wire/
├── aandrijving/
├── mcu/  
└── docs/ (bevat systeemarchitectuur en ontwerpdocumentatie)  

```

## Gebruik

1. Verbind de microcontrollers volgens het aansluitschema in de `README.md` van het project dat u wilt gebruiken. 
2. Flash de software naar de bijbehorende STM-microcontroller met behulp van STM32CubeIDE en een ST-Link programmer.


## Bijdragen

Bijdragen zijn van harte welkom! Volg deze stappen om bij te dragen:

1.  Fork deze repository.
2.  Maak een nieuwe branch aan voor je wijzigingen.
3.  Commit je wijzigingen en push ze naar je fork.
4.  Open een pull request en beschrijf je wijzigingen.

## Licentie

Dit project is gelicenseerd onder de MIT-licentie. Zie het LICENSE-bestand voor meer informatie.



