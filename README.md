# HAN_FlexibleFuelCellStack

This is a short guide to reading data from the Flexible Fuel Cell Stack (FFCS).
The script of Main_Communication_Nano must be uploaded on the arduino nano.

To read the data the arduino IDE can be used by viewing the serial output in the serial monitor, this data can then be coppied to a csv.

Otherwise the python script can be used to create a csv file with the data
To do this a small setup is necessary:
    1. pyhton must be installed. Use -> www.python.org
    2. pySerial also needs to be installed. open the terminal and write: python -m pip install pyserial
    3. Sadly, when using pySerial, the name of the port that the Arduino is connected to must be specified in the code FFCS_read.py in line 11. This description can be found in the Arduino IDE under “Tools → Port”

To log the data follow these steps:
    1. Make sure the arduino IDE is closed, Two programs cannot access the same Serial port.
    2. Open the terminal and start the pyhton script with: python3 FFCS_read.py
    3. The program will run and print out the values received from the Nano in the terminal window. To stop logging data write exit() or quit(). After the programm is stoped, in the same directory as FFCS_read.py, there will now be a file ‘FFCS_log.csv’.

