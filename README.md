# BUESte

## Getting parts:
Parts have been put into Reichelt.de parts baskets. Select a basket, specify the number of times you need the parts and put them in your shopping basket.
The following parts lists are available

- [bue: main pcb](https://www.reichelt.de/my/1827413)
- [bue: button boxes](https://www.reichelt.de/my/1832678)
- [bue: detectors](https://www.reichelt.de/my/1832677)
- [bue: cables](https://www.reichelt.de/my/1832680) select the cables suitable for your setup

Lists are also stored [here](https://github.com/Kleinbahner/BUESte/tree/main/doc/reichelt)

## Manuals:
Manual for main pcb assembly: [main pcb manual](https://github.com/Kleinbahner/BUESte/blob/main/doc/Manual%20(DE)%20Bahn%C3%BCbergang%20Platine%20v2.pdf)

Manual for boxes: [boxes manual](https://github.com/Kleinbahner/BUESte/blob/main/doc/Manual-DE-Box.pdf), [drilling template](https://github.com/Kleinbahner/BUESte/blob/main/doc/drill%20template%20bue.pdf)

Manual for detector: to follow

## Software

Manual for programming the Arduino: [Arduino programming](https://github.com/Kleinbahner/BUESte/blob/main/doc/Programmierung%20Arduino%20f%C3%BCr%20Bahn%C3%BCbergangssteuerung.pdf)

## Components
The railway crossing control unit consists of different components that can can be combined in various ways in order to suit the actual module layout requirements. The components will be described in more detail in the following sections.

### Mainboard

The main board is permanently installed near the railroad crossing. All other components are connected to the main board.

![Main board v1](/img/BUE_Mainboard_complete.jpg) ![Main board v2](/img/BUE_Mainboard_complete_v2.jpg)

The flashing light signals for the railway crossing and the servos for the fences can be connected via terminal blocks. The control inputs are connected to further terminal blocks. There is also an amplifier for connecting a loudspeaker. The sounds can be stored on a microSD card. Power is supplied via an USB socket.

The two RJ45 sockets are used to connect the other components via a looped-through cable. All components can be connected to the same cable in any order. Star cabling is also possible. Alternatively, the inputs and outputs can also be connected directly to the terminal blocks if they are installed in the same module.

#### Pin assignement
All components must be connected to the main board. It has a number of screw and plug connections for this purpose. Ideally, the main board should be mounted near the railroad crossing itself so that many components such as the flashing signals, the light barrier for clear signaling and the loudspeaker can be connected directly. The following illustration provides an overview of the connection options.

![Pin assignement main board v1](/img/BUE_Mainboard_Pin-assignement.png) ![Pin assignement main board v2](/img/BUE_Mainboard_Pin-assignement_v2.png)

The available signals and their meaning are listed here in detail. The output signals for the LED are driven with 5V. Corresponding series resistors must be added yourself!

| Signal | Direction | Activation | Purpose |
| ------ | --------- | ---------- | ------- |
| Loudspeaker | output | - | Connections for a small passive loudspeaker, maximum 1W |
| Servo_1 ... Servo_8 | output | Servo connections for the fences. Pin assignement (from boarder of PCB): - , +, pulse |
| UT-LED_1 | output | LED is on if the current sensor on the left track is disabled by the UT push button |
| UT-LED_2 | output | LED is on if the current sensor on the right track is disabled by the UT push button |
| Uebsig_1 | output | LED for the flashing signals of the left track |
| Uebsig_2 | output | LED for the flashing signals of the right track |
| Blink_1 ... Blink_4 | output | Flashing signals for the railroad crossing.  Blink_1 and 2 are phase shifted to Blink_3 and 4 |
| Bus_1 | output | Signals to attached remote components on the left track |
| Bus_2 | output | Signals to attached remote components on the right track |
| Strom_1 | input | LOW active | Input for the current sensor of the left track |
| Strom_2 | input | LOW active | Input for the current sensor of the right track |
| Lichtschranke | input | HIGH active | Input for the light barrier for clear signaling of the railroad crossing |
| UT_1 | input | LOW active | Input to temporary deactivate the current sensor on the left track |
| UT_2 | input | LOW active | Input to temporary deactivate the current sensor on the right track |
| RS | input | LOW active | Input for the "Rangierschalter" (RS) |
| AT | input | LOW active | Input for the disable push button (AT) |
| ET | input | LOW active | Input for the enable push button (ET) |
| USB Power | input | - | Power supply for the circuit. A power bank is recommended to be independent of outlets. |
| 5V | output | - | Supply voltage for light barrier |
| Masse | - | - | Common potential for push buttons |
| Config (only v1_0) | - | - | Actually not used. Can be used for further extensions. |
| I2C (only v2_0) | - | - |  I2C bus of the arudino. For further extensions. |
| Erweiterung 1 (only v2_0) | - | - | Unused pins of the arduino. Actualy not used. |
| Erweiterung 2 (only v2_0) | - | - | Unused pins of the arduino. Actualy not used. |
| SD-Karte | - | - | Slot for SD card. The card has to contain the WAV file for the signal sound. Attention: There has to be a card in the slot with a WAV file called "ding.wav" in the main directory. Otherwise the control unit will not work! |

#### Assignement of the bus cable
A bus is used to connect the components further away from the main board. All the required signals are routed in a single multi-core cable. An RJ45 cable is used for this. Starting from the main board, all components are connected to each other. The boxes have three sockets and can therefore be used as splitters. All sockets are of equal value. The cabling can be carried out in any combination. Only the left and right sections need to be differentiated as the disable buttons, the sensors and the supervision signals are direction dependent. Examples of wiring can be found below under examples.

The socket is assigned as follows (view into the socket):

![Bus socket](/img/BUE_RJ45-Plug.png)

| Pin | Function |
| --- | -------- |
| 1 | Ground |
| 2 | Enable push button (ET) |
| 3 | Disable push button (AT) |
| 4 | Shunting switch (RS) |
| 5 | Disable current sensor push button (UT) |
| 6 | LED to mark disabled current sensor |
| 7 | LED for supervision signal. Blinks in phase with flashing light signal |
| 8 | Current sensor |

### Enable box
For manual operation of the railraod crossing a small box is needed. It contains the Enable (ET) and Disable (AT) push buttons as well as a shunting switch. The shunting switch can be equiped with an LED that blinks in phase with the flashing light signal to mark its use.

![Einschaltbox](/img/BUE_ET_Box.jpg)

The box can be attached to modules using the well know FREMO-Clamps. The box contains a hole for that purpose.

The connection from the box to the mainboard is through a RJ-45 cable (straight) as it is known from computer networks. Each box has three equal sockets. Thus it can also be used as splitter for cabling.

![Backside of box](/img/BUE_Distribution_Box.jpg)

To ease the cabling inside the box, a small PCB has been designed. It fits perfectly in the boxes used and provides space for three RJ-45 sockets. In addition it provides soldering points for the push buttons and the LEDs.

![PCB for boxes](/img/BUE_Box-PCB.jpg)

### Current sensor
The current sensor is based on the design of Reinhard Müller [http://dcc-mueller.de/wire4dcc/sensor_d.htm](http://dcc-mueller.de/wire4dcc/sensor_d.htm), that has been expanded with a RJ-45 socket for the crossing cabling system.

![Current sensor](/img/BUE_Currentsensor.jpg)

The purpose of the current sensor is the detection of an approaching train. In order to do that one rail of a module is isolated from the neigboring modules (module interconnection cable not pluged in). This rail is then supplied through the current sensor that is attached to the neigboring module. There are no changes of the modules required.

### Sensor box
Near to the current sensor, a sensor box can be attached to the modules using FREMO-clamps. The box can also serve als signal splitter.

![Sensor box](/img/BUE_UT_Box.jpg)

The main purpose of this box is to provide the disable button near to the current sensor. By pressing the push button, the current sensor will be disabled for a give time period. A disabled current sensor is signaled by the LED in the box. Deactivation of the sensor can be necessary if there are shunting activities ongoing that would touch the module with the sensor without the need to close the railway crossing.

### Supervision signal
The engine driver needs to know if the railraod crossing is correctly enabled. That is the purpose of the supervision signal that is placed after the current sensor that turns the railroad crossing on. If the railroad crossing is enabled the signal will be blinking.

![Supervision signal](/img/BUE_Signal.jpg)

The supervision signal is placed in a Wattenscheider signal carrier and thus can be placed in any module with the propper Wattenscheiderslot. The signal is connected to the main board using an RJ45-socket. If an additional marker light should be realised, there has to be a sperate power supply. This can for example be done by including a battery and a voltage regulator in the signal carrier.

## Module setup
The components presented before allow for a flexible setup. Two example setups will be presented here.

### Railraod crossing on open track
The most simple case is a railway crossing on clear track. There the main board is integrated in the module which contains the railway crossing and crossing lights. The flashing signal lights, maybe existing fences and the speaker are then directly connected to the main board. Left and right of the railraod crossing on the module at a sufficient distance a sensor module is used. One rail will be isolated from the other modules and will be supplied through a current sensor. The sensor is connected to the main board using RJ45-cables.

![Setup at free track](/img/BUE_Scheme_Track.png)

To connect the supervising signals it is recommended to install a sensor box near to every current sensor. The boxes are then used as splitters to connect both the current sensor and the signal.
In this setup the railroad crossing is controlled only by the trains, no interaction of the driver is required.

### Railroad crossing at the end of a station
The situation gets more complex if the railroway crossing is located at the end of a station where trains stop. In this case the railroad crossing can not only be automatically engaged by a current sensor at the side of the station. This could be the case if a train approaches to station to make sure that the crossing is stil secured if the train does not correctly stop at the platform.

![Setup in station](/img/BUE_Scheme_Station.png)

As soon as the trains stops, the railway crossing will be deactivated manually using the Einschaltbox to allow road users to use the crossing when the train is stooped at station. Before the trains continue their journey, the crossing has to be enabled again manually. Therefore an Einschaltbox is required.
In addition the box contains the shunting switch. This is switched on if there are shunting activities ongoing that will involve the railway crossing. The lights are then permanently switched on and will only turn off after the shunting switch has been switched off.



