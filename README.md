# Remote-thermometer-nRF52

The project of the wireless thermometer deals with a problem of direct temperature measurements in the rotor of an electric motor. The designated thermometr can measure temperature at up to four places in the motor with average accuracy of 0.8 % from range +20 to +200°C. Main used chips are nRF52833 from Nordic Semiconductor and MAX31865 from Maxim integrated. Temperature is sensed by RTD sensors (PT100 or PT1000). 

The device is powered by small 1/2 AA battery and measured data is sent using the standardised Bluetooth Low Energy protocol so that the measuring system can be connected to a PC as well as to a laptop or smartphone with the appropriate client applications. No special receiver is not required.

The system as a whole is characterised by high noise immunity, low power consumption, high accuracy and mechanical robustness, which has been measured up to a speed of 6500 RPM.

Designed PCB:
![PCB 3D Print](https://user-images.githubusercontent.com/75492624/162914758-d32df542-861a-4f15-93cd-ed19ef844b2e.png)


The project was devoleped as Master thesis project and the thesis is avaible [here](https://www.vut.cz/en/students/final-thesis/detail/142726).
