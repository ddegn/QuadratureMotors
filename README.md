# QuadratureMotors
Code to read from quadrature encoders and to control motors with PWM.
The object "QuadratureMotors" can use any number of quadrature encoders (within reason) and control motors with PWM. The PWM signals are produced by comparing times in a PASM loop so the PWM resolution will depend on the PWM frequency and the number of motors with encoders in use.
