ITU BLE Measurement Service
===========================
Using
- RFDuino [Nordic nRF51288 chip] hardware
- Softdevice 110 v. 521
- Keil uVision5

See this video: https://www.youtube.com/watch?v=FMdcDRUT45g

The following depicts the measurement service with the two characteristics:

![](https://raw.githubusercontent.com/EnergyFutures/ITU_BLE_Measurement_Service/master/img.jpg)


The Value characteristic is only for "presentation": (Read and Notify)

The Config characteristic is for configuration: (Read & Write)
- How the value characteristic is presented (which fields are transmitted)
- To override specific fields, e.g. to set a new sampling freq or change the sequence number

We use the IEEE Float for the measurement value.. fx.

The temperature 25.50 celsius = (Exponent: -2, Mantissa: 2550)

This value is always present.
