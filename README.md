# version 1.0	initial version  October 2018 2017

Copyright (c) 2018 Paul van Haastrecht <paulvha@hotmail.com>


## Background
This project is  part of  a number of projects to measure the air quality I run into DHT11 sensor, 
which can measure temperature and humidity.

The board is the Xinda or Keues. It is available under many different brands, but the DHT11 is the same.

The sensor are well documented and there is software available for Arduino. I have studied that code and 
made a version for the Raspberry PI.  The challenge is capturing all the bits from the device in one read. 
See driver explanation

## Software installation

Make your self superuser : sudo bash
3.1 BCM2835 library
Install latest from BCM2835 from : http://www.airspayce.com/mikem/bcm2835/

1. wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.56.tar.gz
2. tar -zxf bcm2835-1.56.tar.gz		// 1.56 was version number at the time of writing
3. cd bcm2835-1.56
4. ./configure
5. sudo make check
6. sudo make install

3.2 DHT software
Obtain the latest version from : https://github.com/paulvha/dhtxx-on-raspberry


1. Download the zip-file (clone or download / download zip-file) in the wanted directory
2. unzip dhtxx-on-raspberry-master.zip (*1)
3. cd dhtxx-on-raspberry-master
4. create the executable : make
5. To run you have to be as sudo ./dht -h ….

*1) if you do not have unzip : sudo apt-get install zip unzip

(detailed description of the many options in  dht.odt in the documents directory)

