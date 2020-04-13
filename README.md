### Arduino library for communicating with the MPU-6050 six-axis Inertial Measurement Units (IMU).

#The gyroscope and accelerometer has 16 bits of resolution. The gyroscopes measures up to +- 2000 degrees per second. The accelerometers measures up to +-16g.
InvenSense provides product specification and register map documents, enough to figure out how to interface with the chips. I can ignore the  advanced features like the FIFO buffer, interrupts and the digital motion processor. I just need to configure the chips, read the gyroscope and accelerometer values.

##                               Alpha version
