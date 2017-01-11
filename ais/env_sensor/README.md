## UiEM01 Environmental sensor under linux

 This node connects to a UiEM01 Version 1.0 and publishes instantaneous sensor information.
 UiEM01 contains the following sensors:
 
 CO        
      Sensor: Alphasense CO-AF
      Unit: ppm
      
      CO Measures based on CO-AF should be compensated. Sensor output decais on time.
      Currently there is no method to do this properly.
      
 Light
      Sensor: ROhm bh1710fvc-e
      Unit: lux
      
 Particle Count
      Sensor: Shinyei AES-1
      Unit: particles/0.01CF
      
 Temperature and Humidity   
      Sensor: Sensirion SHT11 Sensor
      Units: Celsius degrees and %
      
 Volatile Organic Compounds 
      Sensor: TGS2602
      Unit: KOhms

Before using it, install.sh must be executed with root privileges to set up serial access.
