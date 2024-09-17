For an egg to hatch, it must be incubated at a temperature close to 38 degrees for about 21 days.

This machine creates an environment close to 38 degrees for 21 days to provide an environment for eggs to hatch. 


The machine has three states. Preparing, Incubating and Completed.

The states are controlled by a switch on the breadboard which, when pressed, changes from the current state to the next.

In the Preparing and Completed states, the heater, temperature probe and time calculation are disabled, and only in the Incubating state are all these features enabled.

As mentioned earlier, you can enter the Incubating state via a switch.

When the MCU is first powered up, the OLED display shows an introduction and logo, and from then on it is continuously updated to show the status of the machine (current status, temperature, heater on/off and time-related information).

When entering incubation mode, the temperature sensor inside the box starts to read the temperature and if it is below 38 degrees, the heater is activated to raise the temperature inside the box to 38 degrees.
If it rises above 38 degrees, the heater stops working.

The behaviour of the heater has been implemented using an AC voltage source and a relay. See the link below for more information.
https://samlee.hashnode.dev/controlling-heater-ac-relay

This behaviour will continue for 21 days, after which the mode will change to completed.

Translated with DeepL.com (free version)
