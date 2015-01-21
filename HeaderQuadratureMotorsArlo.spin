DAT objectName          byte "HeaderQuadratureMotorsArlo", 0
robotName               byte "Arlo Hardware with Hb25", 0
  
CON
{{
  ******* Public Notes *******
  
}}
{
  ******* Private Notes *******

  HeaderVexTreads130130a modified DemoFourMotorsAndEncoders130130a
  130130a Move individual robot constants to a header file
  to make it easy to use the same code with multiple robots.
  130201a Set some unused pins to "-1".
  07a Motor #0 move robot in reverse with positive speed.
  Motor #1 move robot forward with positive speed.
  1208a Modify to match changes made when servo used.
  Change name from "HeaderFaulhaber140607a" to "XHeader150116a".
}
{{
   Faulhaber Gear Motor Information

     6  4  2

     5  3  1

      ||||||  ribon cable side
      

  1. Motor Positive
  2. 5V Encoder Vdd
  3. Encoder A
  4. Encoder B
  5. Encoder Ground
  6. Motor Negative
  
  
     
}}   
CON '' Activity Board Constants
  
  ' Encoders
  ENCODERS_PIN = 14

  ENABLE_0 = 12
  ENABLE_1 = 13
 
  ' ADC (Activity Board)
  ADC_CS = 21                                        
  ADC_SCL = 20 
  ADC_DO = 19 
  ADC_DI = 18

  
CON
  '' The "RED" constants don't have to be assigned to
  '' red wires. It may be useful to switch the pin
  '' assignments if the wheel direction doesn't match
  '' the encoder directions.
  '' Set FIRST_ENCODER_x to "-1" if a encoder isn't being used.
  '' Set WHEEL_ENABLE_x to "-1" if a motor isn't being used.
  '' If WHEEL_RED_x is set to -1 and the enable pin is set
  '' to a valid I/O pin, the motor will be treated as a
  '' hobby servo.
  '' If WHEEL_RED_x is a valid I/O pin but WHEEL_BLACK_x
  '' is set to -1, then the motor will be treated as
  '' being controlled by a h-bridge with a single direction
  '' control pin. Dynamic braking will not be possible
  '' in this case.

  WHEEL_RED_0 = -1
  WHEEL_BLACK_0 = -1
  WHEEL_RED_1 = -1
  WHEEL_BLACK_1 = -1
 
  
  WHEEL_ENABLE_0 = ENABLE_0 
  WHEEL_ENABLE_1 = ENABLE_1 
   
  FIRST_ENCODER_0 = ENCODERS_PIN
  FIRST_ENCODER_1 = ENCODERS_PIN + 2
 
  
  FORWARD0 = 1
  FORWARD1 = 1
   
  
  '4331 transitions per revolution ** wrong ** info from Pololu incorrect
  'Gears in Pololu motor: 10:25,10:30,12:28,10:40
  ' reduction = 25/10 * 30/10 * 28/12 * 40/10 = 5/2 * 3 * 7/3 * 4 = 5*7*2 = 70
  ' 70 * 64 = 4,480 transitions per revolution

  'Parallax 30:1 gearmotor have 24 transitions per revolution of motor with
  ' 64 * 30 = 1920 transitions per revolution drive shaft.
   
  'MAX_TRANSITIONS_PER_SEC = 125 '12_300 '10827 '282
  MAX_TRANSITIONS_PER_SEC = 125 ' Arlo hardware

  'MAX_TRANSITIONS_PER_SEC = 500 ' Dagu Rover 5 
  ' 1000 transitions per three rotations  1.5 rev per second

  'TRANS_PER_REV_NUMERATOR = 4480 ' Pololu 70:1 gearmotors (listed as 67:1) 
  'TRANS_PER_REV_DENOMINATOR = 1 ' In case transitions per rev isn't a whole number         
  'TRANS_PER_REV_NUMERATOR = 1920 ' Parallax 30:1 gearmotors (listed as 67:1) 
  'TRANS_PER_REV_DENOMINATOR = 1 ' In case transitions per rev isn't a whole number         
  'TRANS_PER_REV_NUMERATOR = 1000 ' Dagu Rover 5 
  'TRANS_PER_REV_DENOMINATOR = 3 ' Dagu Rover 5      
    'TRANS_PER_REV_NUMERATOR = 22512 ' Faulhaber gearmotors (listed as 140.759183:1)(measured 140.70 
  'TRANS_PER_REV_DENOMINATOR = 40 ' In case transitions per rev isn't a whole number
  'TRANS_PER_REV_NUMERATOR = 56_308 ' Faulhaber gearmotors (listed as 140.759183:1)(measured 140.77 
  'TRANS_PER_REV_DENOMINATOR = 100 ' In case transitions per rev isn't a whole number
  TRANS_PER_REV_NUMERATOR = 144 ' Arlo hardware
  TRANS_PER_REV_DENOMINATOR = 1 ' In case transitions per rev isn't a whole number
        
  '4082/29 = 140.7586206897
  'DIAMETER_OF_WHEEL = 66 ' ActivityBot wheel in mm
  'DIAMETER_OF_WHEEL = 92 ' Large Vex Omni wheel in mm
  'DIAMETER_OF_WHEEL = 69 ' Small Vex Omni wheel in mm
  DIAMETER_OF_WHEEL = 152 ' Arlo hardware
  
 ' PSEUDO_RADIUS_OF_WHEEL = (DIAMETER_OF_WHEEL * PSEUDOREAL_MULTIPLIER) / 2                     
                      
  PSEUDOREAL_MULTIPLIER = 1000 ' If the multiplier is too large values used in calculations
                              ' could exceed 32-bits.

  {
  #0, NO_LIMIT, SUBTRACT_REVOLUTON_LIMIT, HARD_STOP_LIMIT
  
  LIMIT_TYPE_0 = NO_LIMIT 'SUBTRACT_REVOLUTON_LIMIT
  LIMIT_TYPE_1 = NO_LIMIT
  LIMIT_TYPE_2 = NO_LIMIT
  LIMIT_TYPE_3 = NO_LIMIT

  LIMIT_LOW_0 = (TRANS_PER_REV_NUMERATOR * -3) / (TRANS_PER_REV_DENOMINATOR * 2)
  LIMIT_LOW_1 = 0
  LIMIT_LOW_2 = 0
  LIMIT_LOW_3 = 0

  LIMIT_HIGH_0 = (TRANS_PER_REV_NUMERATOR * 3) / (TRANS_PER_REV_DENOMINATOR * 2)
  LIMIT_HIGH_1 = 0 
  LIMIT_HIGH_2 = 0
  LIMIT_HIGH_3 = 0

  PSEUDO_MIN_SPEED_0 = -1000 * PSEUDOREAL_MULTIPLIER
  PSEUDO_MIN_SPEED_1 = -1000 * PSEUDOREAL_MULTIPLIER
  PSEUDO_MIN_SPEED_2 = -1000 * PSEUDOREAL_MULTIPLIER
  PSEUDO_MIN_SPEED_3 = -1000 * PSEUDOREAL_MULTIPLIER

  PSEUDO_MAX_SPEED_0 = 1000 * PSEUDOREAL_MULTIPLIER
  PSEUDO_MAX_SPEED_1 = 1000 * PSEUDOREAL_MULTIPLIER
  PSEUDO_MAX_SPEED_2 = 1000 * PSEUDOREAL_MULTIPLIER
  PSEUDO_MAX_SPEED_3 = 1000 * PSEUDOREAL_MULTIPLIER
  }
  
  DEFAULT_ACCELERATION = 100
  PSEUDO_ACCELERATION = DEFAULT_ACCELERATION * PSEUDOREAL_MULTIPLIER '1000 '
  REFRESH_RATE = 50 '20 '
  
  'PID_INCREASE = 10
  'PID_MULTIPLIER = PSEUDOREAL_MULTIPLIER * PID_INCREASE
 
  'PROPORTIONAL_CONSTANT = 0 '40 '1 '24 '5
  'INTEGRAL_CONSTANT = 0 '4
  'MAX_INTEGRAL = 2_000_000 '2000
  'NEW_INTEGRAL_NUMERATOR = 0 '300
  'NEW_INTEGRAL_DENOMINATOR = 1000
  '' "NEW_INTEGRAL_NUMERATOR" and "NEW_INTEGRAL_DENOMINATOR" are used when adding the integral
  '' sum.
  
  'DERIVATIVE_CONSTANT = 0 '2
  'DEFAULT_INTEGRATOR_LEAK = 0 '(PSEUDOREAL_MULTIPLIER * 95) / 100 ' not presently used.
  
  {STEPS_PER_FIGURE_8 = 1150
  INPUT_SIGNIFICANT = 20    ' used to detect when landing gear switch as been thrown
  
  ' wheelMode enumeration
  #0, INDEPENDENT_WHEELS, TREADED_WHEELS, MECANUM_WHEELS 

  'WHEEL_MODE = INDEPENDENT_WHEELS '' Use this when first setting up robot
  'WHEEL_MODE = TREADED_WHEELS     '' The target speeds of each side will be matched.
  WHEEL_MODE = MECANUM_WHEELS    
  
  ' inputType enumeration
  #0, PC_ONLY_INPUT, WII_NUNCHUCK_INPUT, RC_INPUT, WII_AND_RC_INPUT, AUTO_DETECT_INPUT

  'INPUT_TYPE = PC_ONLY_INPUT       '' Use this when first setting up robot
  'INPUT_TYPE = WII_NUNCHUCK_INPUT   
  'INPUT_TYPE = RC_INPUT
  'INPUT_TYPE = WII_AND_RC_INPUT    '' Not yet supported. If used it will behave the same
                                    '' as AUTO_DETECT_INPUT.                                                                                                                                            
  INPUT_TYPE = AUTO_DETECT_INPUT   '' Priority given to Nunchuck
  
  DEFAULT_SPEED = 150 '450 '600    ' Used in figure 8 not presently calibrated
                              
  DEFAULT_TURN_RATE = 100 '25 'DEFAULT_SPEED / 2 '4  ' Used in figure 8, the larger the turn rate 
                                            ' the larger the loops of the figure 8 will
                                            ' be.
                                            
  ' inputType enumeration 
  #0, NO_COMPASS, HMC5883_COMPASS, AUTO_DETECT_COMPASS
  
  COMPASS_MODE = HMC5883_COMPASS

  ' phaseOf8 enumeration 
  #0, START_1ST_LOOP, WATCH_FOR_END_1ST_LOOP, EXTEND_1ST_LOOP, {
  } START_2ND_LOOP, WATCH_FOR_END_2ND_LOOP, EXTEND_2ND_LOOP, FINISHED_LOOP
                                              
  ' stepsOf8 
  START_1ST_STEPS = REFRESH_RATE * 2 'STEPS_PER_FIGURE_8 / 8 ' quarter circle
  WATCH_FOR_END_1ST_STEPS = 10000'(STEPS_PER_FIGURE_8 / 2) - START_1ST_STEPS'START_1ST_STEPS * 3
  EXTEND_1ST_STEPS = STEPS_PER_FIGURE_8 / 4
  START_2ND_STEPS = START_1ST_STEPS
  WATCH_FOR_END_2ND_STEPS = WATCH_FOR_END_1ST_STEPS
  EXTEND_2ND_STEPS = EXTEND_1ST_STEPS
  WAIT_FOR_8_TIME = REFRESH_RATE * 4 ' four seconds
  

  MIN_STEPS = STEPS_PER_FIGURE_8
  MAX_STEPS = STEPS_PER_FIGURE_8 * 100
                           
  #0, COUNTER_CLOCKWISE_TURN, CLOCKWISE_TURN
                            
  FIRST_LOOP_TURN_DIRECTION = COUNTER_CLOCKWISE_TURN

  ' slaveMode enumeration
  #0, MASTER, SLAVE 

  DEFAULT_SLAVE_MODE = MASTER

  ' encoderMode enumeration
  #0, COUNT_TICKS_SPEED, TIME_TICKS_SPEED, HYBRID_SPEED 

  DEFAULT_ENCODER_MODE = COUNT_TICKS_SPEED

  ' startAction enumeration
  #0, WAIT_FOR_INPUT_START, FIGURE_8_START, AUTONOMOUS_WANDER_START
  
  DEFAULT_START = WAIT_FOR_INPUT_START

  ' radio type enumeration
  #0, RC_RADIO, NORDIC_NRF24L01P_RADIO, NORDIC_NRF2401A_RADIO

  DEFAULT_RADIO = NORDIC_NRF24L01P_RADIO

  ' pin offsets
  #0, IRQ_PIN_OFFSET, MISO_PIN_OFFSET, MOSI_PIN_OFFSET, SCK_PIN_OFFSET, SCN_PIN_OFFSET, CE_PIN_OFFSET
  }
PUB GetObjectName

  result := @objectName

PUB GetRobotName

  result := @robotName

{PUB GetControllerPins

  case DEFAULT_RADIO
    RC_RADIO:
      result := @controllerPin
    NORDIC_NRF24L01P_RADIO:
      result := @nordicPin
}     
PUB GetEncoderPins

  result := @encoderPin
  
PUB GetEnablePins

  result := @enablePin
  
PUB GetRedPins

  result := @redPin
  
PUB GetBlackPins

  result := @blackPin
  
{PUB GetLimitType

  result := @limitType
  
PUB GetLimitLow

  result := @limitLow
  
PUB GetLimitHigh

  result := @limitHigh
  
PUB GetMinSpeed

  result := @pMinSpeed
  
PUB GetMaxSpeed

  result := @pMaxSpeed
        }
{PUB GetPidParameters

  result := @pidParameters
  
PUB PathData

  result := @autonomous
}  
{DAT

controllerPin byte RADIO_PIN, RADIO_PIN + 1, RADIO_PIN + 2, RADIO_PIN + 3
              byte RADIO_PIN + 4, RADIO_PIN + 5
              
DAT

nordicPin     byte NODIC_IRQ_PIN, NORDIC_MISO_PIN, NODIC_MOSI_PIN
              byte NODIC_SCK_PIN, NODIC_SCN_PIN, NODIC_CE_PIN
}
DAT

encoderPin    byte FIRST_ENCODER_0, FIRST_ENCODER_1
                          
DAT

enablePin     long WHEEL_ENABLE_0, WHEEL_ENABLE_1

DAT

redPin        long WHEEL_RED_0, WHEEL_RED_1

DAT

blackPin      long WHEEL_BLACK_0, WHEEL_BLACK_1

{DAT

limitType     long LIMIT_TYPE_0, LIMIT_TYPE_1, LIMIT_TYPE_2, LIMIT_TYPE_3

DAT

limitLow      long LIMIT_LOW_0, LIMIT_LOW_1, LIMIT_LOW_2, LIMIT_LOW_3

DAT

limitHigh     long LIMIT_HIGH_0, LIMIT_HIGH_1, LIMIT_HIGH_2, LIMIT_HIGH_3

DAT

pMinSpeed     long PSEUDO_MIN_SPEED_0, PSEUDO_MIN_SPEED_1, PSEUDO_MIN_SPEED_2, PSEUDO_MIN_SPEED_3

DAT

pMaxSpeed     long PSEUDO_MAX_SPEED_0, PSEUDO_MAX_SPEED_1, PSEUDO_MAX_SPEED_2, PSEUDO_MAX_SPEED_3
    }

{DAT
      
pidParameters long PROPORTIONAL_CONSTANT, INTEGRAL_CONSTANT, DERIVATIVE_CONSTANT, PID_INCREASE
              long MAX_INTEGRAL
              
DAT
     
autonomous    long DEFAULT_SPEED
}