DAT programName         byte "QuadratureDemo", 0
CON
{{
  ******* Public Notes *******

  Author: Duane Degn
  Date First Released: December 10, 2013
  Date Latest Released: December 12, 2013

  MIT License

  The current object being demonstated has
  a resolution of 6us (at 80MHz). The main PASM
  loop takes 480 clock cycles to complete.
  It may be possible to increase this resolution
  if only two motors/encoders are used and
  to code controlling and reading the unused
  motors and enocoders is removed. 
  
-------------------------------------------------  
  Motor Control Information:
-------------------------------------------------  
  The "FourQuadratureMotor" object can control
  motors requiring normal PWM input as well
  as continuous rotation servos.

  I have not tested the precision of the servo
  control signal. I doubt it has 1us resolution
  as the Servo32vX object does. If you have
  cogs available, you may wish to use a
  different object than "FourQuadratureMotor"
  for continuous rotation servo control.
  
  The program can not currently use both normal
  h-bridge motors and continuous rotation
  simultaneously. All motors are assumed to be 
  the same type. The program can use both
  types of direction control features. A
  h-bridge with single pin direction control
  may be used together with a dual pin control
  h-bridge.

  Set WHEEL_ENABLE_x to -1 if no motor used on
  that channel.
  
  If WHEEL_ENABLE_x if set to a valid I/O pin
  and WHEEL_RED_x is set to -1, the motor will
  be treated as a continuous rotation servo.

  If WHEEL_ENABLE_x and WHEEL_RED_x are both
  set to valid I/O pins and WHEEL_BLACK_x is
  set to -1, the WHEEL_RED_x pin will be used
  to set the direction of the motor. High for
  forward, low for reverse. Your h-bridge must
  support this type of interface in order to
  use it. This interface does not allow
  dynamic braking.
  
  If WHEEL_ENABLE_x, WHEEL_RED_x and
  WHEEL_BLACK_x are all set to valid I/O pins,
  WHEEL_RED_x and WHEEL_BLACK_x pins will be
  used to set the direction of the motor.
  Forward speeds will set WHEEL_RED_x high
  and WHEEL_BLACK_x low. The high and low
  states are switched for reverse.
  When speed is set to zero both direction
  pins will be set low.

-------------------------------------------------  
  Encoder Information:
-------------------------------------------------  
  The algorithm used in the encoder object
  comes out of JonnyMac's Spin Zone article
  "Spinning it Up with Encoders".

  Up to four quadrature encoders may be used
  with the "FourQuadratureMotor" object. The
  "A" and "B" sensor pins need to be
  consecutive I/O pins. Only the first of the
  two pins is listed in the "FIRST_ENCODER_x"
  section of the header object. Provisions
  have been made to reverse the direction
  of the encoder feedback if desired without
  the need to physically swap the wires
  connected to the Propeller. See comments
  within the program about setting the
  encoder direction.

  If an encoder is not used the "FIRST_ENCODER_x"
  pin should be set to -1.

  If an encoder pin is set to a valid I/O pin
  but the I/O pin is left to float, the input
  pins may register phantom transitions.

  There are several constants in the header
  object used to compute speeds.

  The constant "MAX_TRANSITIONS_PER_SEC" is
  used to scale the speed in order to have
  the measured speed have the same range as
  the PWM resolution. For example, if the
  "DEFALUT_PWM_RESOLUTION" constant in the
  child object is set to 1000, then a speed
  setting of 1000 should produce a measured
  speed of 1000. In order for the measured
  speed to match the input speed the constant
  "MAX_TRANSITIONS_PER_SEC" needs to be set
  to the appropriate value.
  
  This appropriate value may be calculated
  based on the motors known max rpm and
  transitions per revolution data or it
  may be directly measured by running this
  program while the motor runs at full
  speed. 

  Rather than having a single constant for
  the number of transitions per revolution
  of the motor, I used two.
  "TRANS_PER_REV_NUMERATOR" and
  "TRANS_PER_REV_DENOMINATOR".
  This allows for non-integer encoder values.
  For example the Rover 5 has encoders which
  register 1000 transitions every 3 rotations
  of the wheel. Expressing the encoder
  resolution as a fraction allows this 1000/3
  resolution to be accurately stated with
  integers. 

  There are two direction arrays,
  "encoderDirection" and "directionCorrection".
  A pointer to the "encoderDirection" array
  is passed to the encoder object which
  populates the array with ones or negative
  ones based on the sensed direction the
  encoders are turning. The other array
  "directionCorrection" is filled with
  constants from the header object. These
  constants are used to correct the direction
  reported by the encoders so the wires
  of the encoders do not need to be physically
  swapped.
  
  
-------------------------------------------------  
  Features To Add:
-------------------------------------------------  
  Fix RPM.
  Add ability to move a set about of distance.
  Add "jerk" feature to allow acceleration to
  be ramped.
 
}}
CON
{
  ******* Private Notes *******

  131210b "RPM_SPEED_MULTIPLIER" no longer need
  now that "integerMathEnhancer[0]" is being used.
  "RPM_SPEED_MULTIPLIER" now causes 32-bit overflow.
  10c Is working better but only a tenth of an
  rpm unit of accuracy.
  10g Try to maximize resolution of speed.
  10g Some sort of timing problem. It's not
  registering stopped consistently.
  10h Seems to work okay.
  10i Add divide with rounding method.
  10i The divide with rounding didn't change the
  values much. If there was a difference it was
  only in the very last digit. I don't think
  rounding is worth the time delay.
  10j Remove rounding method and clean up
  formatting a bit.
  10j Spell check comments.
  10k Simplify a few constants.
  11b Fix direction problem
  11b Break the debug process into multiple
  methods.
  12f Added speed input seems to work.
  Added a bunch of display code.
  13a Start adding PID control.
  13c Improved some of the formatting.
  13c Still not feedback control.
  13c Stack space is now monitored and the control
  cog is watched to make sure it doesn't freeze.
  13g Try to make each parameter have it's own
  increment and decrement settings.
  13h May have working PID. The output isn't displayed
  so it's hard to tell if the feedback is doing anything.
  13i Output not being set.
  14a Move menu to constant section.
  14a Start adding PID debug info.
  14a Does not compile.
  14c Kind of works. I don't like the way the
  PID portion behaves.
  14d Try to use a circular buffer for integral
  portion of PID.
  14d PID has problems. I don't think I'm using
  the buffer correctly. I'm treating it as a
  buffer of bytes.
  14e There is something wrong with the way
  speed feedback is stored. Channel 1 speed appears
  to match channel 0 speed no matter the output.
  15a Fix where scaled speed is calculated.
  16b Use lookup to find active parameter.
  16c Move small and large increment amounts to
  the DAT section in order to simplify code.
  These amounts will not be adjust able at
  runtime.
  16c The data entry seems to work alright but the
  motors aren't behaving correctly. Motor #1 doesn't
  respond the same way motor #0 does.
  16d There was a problem with the scale speed
  method.
  17a Try a large integrator buffer again.
  17a Add ability to change frequency and resolution.
  17a The frequency seems to change correctly but the
  resolution doesn't seem right.
  17b Experiment with integrator.
  17b Still not working well at low speeds.
  I think the integral component needs to be adjusted.
  17c Add new ways to adjust integral portion and
  remove means to adjust incremental adjustments.
  17d Try to make it possible to adjust buffer
  size in child object.
  17d Trouble with several features.
  18a Change order of methods so all methods run in
  cog #0 are at the beginning of the program.
  18a I also moved the DAT section from the end of
  the program to the end of the cog #0 code.
  I'm not sure if I like the DAT section the way it
  is after the move.
  18b Make the control loop refresh rate one of the
  modifiable parameters.
  18b Compiles but not tested.
  19a Switch to testing on Mecanum wheeled robot.
  19a I'm not sure if program is working correctly.
  I'm going to switch back to the other motors to
  test.
  140427a Start emplimenting Tracy Allen's math
  methods.
  140428a The speed calculations using Tracy's math looks good
  but there are still issues with the PID portion of the code.
  28b I don't think the pseudoScaledSpeed is calculated correctly.
  28c I need a better name for pseudoScaledSpeed and pseudopseudoScaledSpeed.
  Or not. I should just make sure I use "pseudo" when indicating
  a number has been multiplied by a number to increase its
  resolution.
  29a Start using "singlePwmResTicks".
  140529a Start working with Faulhaber motors.
  29b Works with programName commented out but not otherwise.
  30a Problem was the buffer wasn't long aligned.
  Change name from "EncoderDemo140608a" to "XDemo150116a".
  150117a Appears to work.
  18c Abandon 18b
  18e Child objectss 18a through 18d don't work correctly.
  18e Encoder works properly with 18e child.
  18f Still works after small change to child (18f).
  18f Still works. Encoder direction reversed in child object.
  20p Start removing stack size code.
  Move to GitHub ddegn/QuadratureMotors.
}
CON                     
                     
  _clkmode = xtal1 + pll16x 
  _xinfreq = 5_000_000

  CLK_FREQ = ((_clkmode - xtal1) >> 6) * _xinfreq
  MICROSECOND = CLK_FREQ / 1_000_000
  MILLISECOND = CLK_FREQ / 1_000
 {
  DATA_PIN  = 29                 '' user changeable                                 
  CLOCK_PIN = 28                 '' user changeable
  READY_PIN = -1 '26
  DOWN_BUTTON = 16
  UP_BUTTON = 17
  CONTINUOUS_SWITCH = 23
  
  TEMP_MESSAGE_DISPLAY_TIME = CLK_FREQ * 4
  }
  MAX_POSSIBLE_ENCODERS = Encoders#TOTAL_ENCODERS
  ARRAYS_TO_FREEZE = 7
  LONGS_TO_FREEZE = MAX_POSSIBLE_ENCODERS * ARRAYS_TO_FREEZE
  'MAX_POSITIVE = $7FFFFFFF
  'LARGEST_PRACTICAL_MULTIPLIER = MAX_POSITIVE 
  'LARGEST_POWER_OF_TEN_LIKELY = 10000

  LINE_SIZE = 40
  
  'ONE_K = 1_000
  'TEN_K = 10_000

  HEADING_ROWS = 5
  MENU_2ND_COL_LOC_X = 54
  MENU_LOC_Y = HEADING_ROWS
  MENU_ROWS = 0 '11 
  'ACTIVE_PARAMETER_LOC_Y = MENU_LOC_Y + MENU_ROWS + 1
  ACTIVE_PARAMETER_ROWS = 1 '3
  'RESOLUTION_PLUS_LOC_Y = ACTIVE_PARAMETER_LOC_Y + ACTIVE_PARAMETER_ROWS
  RESOLUTION_PLUS_ROWS = 4'5
  'ENCODER_DATA_LOC_Y = RESOLUTION_PLUS_LOC_Y + RESOLUTION_PLUS_ROWS 
  ENCODER_DATUM_ROWS = 5 '4
  ENCODER_DATA_ROWS = ENCODER_DATUM_ROWS * Encoders#TOTAL_ENCODERS
  'IO_PIN_STATES_LOC_Y = ENCODER_DATA_LOC_Y + ENCODER_DATA_ROWS + 1
  IO_PIN_STATES__ROWS = 5
  'PID_DATA_LOC_Y = IO_PIN_STATES_LOC_Y
  PID_DATUM_ROWS = 3
  PID_CONSTANT_ROWS = 2
  'PID_DATA_ROWS = (PID_DATUM_ROWS * Encoders#TOTAL_ENCODERS) + 3
  'TARGET_SPEED_LOC_Y = PID_DATA_LOC_Y + PID_DATA_ROWS 'IO_PIN_STATES_LOC_Y + IO_PIN_STATES__ROWS
  TARGET_SPEED_ROWS = 1 '6
  'SPEED_MESSAGE_LOC_Y = TARGET_SPEED_LOC_Y + TARGET_SPEED_ROWS 
  SPEED_MESSAGE_ROWS = 4
  'TEMP_MESSAGE_LOC_Y = SPEED_MESSAGE_LOC_Y + SPEED_MESSAGE_ROWS
  TEMP_MESSAGE_ROWS = 3
  POSITION_ROWS = 2
  ENCODER_ROWS = 5 '4
  
  DIVIDING_ROWS = 1
  DIVIDING_CHARACTERS = LINE_SIZE
  DIVIDING_CHARACTER = "-"
  
  'DEFAULT_LAST_ROW = 62 '60 '52
  
  DEFAULT_LAST_CHANNEL = 1
  DEFAULT_FIRST_CHANNEL = 0
  
  DEBUG_BAUD = 115200
  QUOTE = 34  ' double quotes
  BELL = 7
  
  ASTRIX_IN_ROW = LINE_SIZE  
  ' activeParameter enumeration                   integratorBuffer
{  #0, PROPORTIONAL_PARAMETER, INTEGRAL_PARAMETER, DERIVATIVE_PARAMETER, {
    } MAX_INTEGRAL_PARAMETER, NEW_I_NUMERATOR_PARAMETER, NEW_I_DENOMINATOR_PARAMETER, {
    } INTEGRATOR_LEAK_PARAMETER, PID_DENOMINATOR_PARAMETER, TOP_SPEED_FEEDBACK_PARAMETER, { 
    } I_BUFFER_SIZE_PARAMETER, ACCELERATION_PARAMETER, CONTROL_RATE_PARAMETER, {
    } BITS_TO_AVERAGE_PARAMETER, FREQUENCY_PARAMETER, RESOLUTION_PARAMETER, {
    } ENCODER_DISPLAY_PARAMETER, TARGET_DISPLAY_PARAMETER, SPEED_DISPLAY_PARAMETER, {
    } ERROR_DISPLAY_PARAMETER, OUTPUT_DISPLAY_PARAMETER, ACCELERATION_DISPLAY_PARAMETER
 
  MAX_PARENT_PARAMETER_INDEX = RESOLUTION_PARAMETER 
  TOTAL_PARENT_PARAMETERS = MAX_PARENT_PARAMETER_INDEX + 1
  MAX_PARAMETER_INDEX = ACCELERATION_DISPLAY_PARAMETER 
  TOTAL_PARAMETERS = MAX_PARAMETER_INDEX + 1
  
DAT

pseudoParameters        long ACCELERATION_PARAMETER, INTEGRATOR_LEAK_PARAMETER, TOTAL_PARAMETERS

decimalPointFlag        byte 0[TOTAL_PARAMETERS]
  
channelParameters       long ENCODER_DISPLAY_PARAMETER, TARGET_DISPLAY_PARAMETER
                        long SPEED_DISPLAY_PARAMETER, ERROR_DISPLAY_PARAMETER
                        long OUTPUT_DISPLAY_PARAMETER, ACCELERATION_DISPLAY_PARAMETER   
                        long TOTAL_PARAMETERS
                        
channelParameterFlag    byte 0[TOTAL_PARAMETERS]
}
CON

  POSITION_BUFFER_SIZE = 25

  SPEED_NUMERATOR = 2_000_000_000 ' largest manageable number
  ' 25 seconds of clock ticks
  ADJ_TO_TICKS_PER_HALF_SECOND = SPEED_NUMERATOR / (CLK_FREQ / 2)
  MIN_SPEED_RESOLUTION = 1000
  MAX_USABLE_TIME = SPEED_NUMERATOR /  MIN_SPEED_RESOLUTION
  '' The values "SPEED_NUMERATOR" and "MANAGABLE_BIT_ADJUSTMENT" are used to
  '' calculate the speed from the time between encoder ticks.
  '' The value produced from this calculation is a number 100 times
  '' as large as the value of the speed calculated with the original
  '' technique.
  '' Hopefully these extra digits are significant and the speed values
  '' calculated from the time between encoder ticks will allow for
  '' better control of the motors.
  
CON

  #0, NORMAL_PARAMETER, PSEUDO_PARAMETER

  #0, NORMAL_PARAMETER_TYPE, CHANNEL_PARAMETER_TYPE
  
  #0, ERROR_MESSAGE, SPEED_MESSAGE, WAIT_FOR_INPUT_MESSAGE, NEW_INPUT_MESSAGE, {
    } ALTERNATE_MESSAGE
  
  MAX_INDEX_TEMP_MESSAGE_TYPES = ALTERNATE_MESSAGE 
  TEMP_MESSAGE_TYPES = MAX_INDEX_TEMP_MESSAGE_TYPES + 1

  CONTROL_LOOP_STACK = 64

  MAX_MESSAGE_SIZE = 100

  MAX_ALLOWED_BUFFER = Header#REFRESH_RATE * 5
  NEXT_TRY_RATE_PERCENT = 99

  #0, SPEED_MODE, POSITION_MODE
  #-1, SET_ONLY_LOCK, NO_ACTION_LOCK, CLEAR_ONLY_LOCK, SET_AND_CLEAR_LOCK
  
VAR

  long controlStack[CONTROL_LOOP_STACK], controlCog
  long measuredStack, previousMeasuredStack
  long controlLoopCounter, previousCounter
  long watchdogTimer, watchdogInterval
  long directionCorrection[MAX_POSSIBLE_ENCODERS]

  ''***
  long output[MAX_POSSIBLE_ENCODERS]
  ''***

  long targetSpeed[MAX_POSSIBLE_ENCODERS], pseudoRampedSpeed[MAX_POSSIBLE_ENCODERS]
  long pseudoError[MAX_POSSIBLE_ENCODERS], previousError[MAX_POSSIBLE_ENCODERS]
  long proportional[MAX_POSSIBLE_ENCODERS]
  long integralSum[MAX_POSSIBLE_ENCODERS], derivative[MAX_POSSIBLE_ENCODERS]
  long zeroSpeedTime
  long newCount[MAX_POSSIBLE_ENCODERS], singlePwmResTicks
  long transPerSecSpeedDisp[MAX_POSSIBLE_ENCODERS], pseudoScaledSpeed[MAX_POSSIBLE_ENCODERS] 
  long rpmSpeed[MAX_POSSIBLE_ENCODERS]', linearSpeed[MAX_POSSIBLE_ENCODERS], linearUnitsPtr 
  'long integerMathEnhancer[2], otherPortionOfEnhancer[2], powerOfTenPortionOfEnhancer[2]
  'long powerOfTenPortionsOfEnhancer, otherPortionsOfEnhancer
  'long parentParameter[TOTAL_PARENT_PARAMETERS], tempMessageTimer[TEMP_MESSAGE_TYPES]
  'long controlAlgorithmRate
  long maxPossibleRefresh, controlAlgorithmInterval
  long debugInterval, negPidMaxIntegral, previousParameterValue
  long pseudoGlobalAccel[MAX_POSSIBLE_ENCODERS]
  long integratorTotal[MAX_POSSIBLE_ENCODERS]
  long integratorBuffer[MAX_ALLOWED_BUFFER * MAX_POSSIBLE_ENCODERS], bufferSize
  'long bufferHead, previousHead
  long longestPasmLoop, controlTimer
  'long newSpeed[MAX_POSSIBLE_ENCODERS]
  long {maxSpeedTicks,} minSpeedTicks
  'long positionLimitType[MAX_POSSIBLE_ENCODERS]
  'long positionLimitLow[MAX_POSSIBLE_ENCODERS]
  'long positionLimitHigh[MAX_POSSIBLE_ENCODERS]

  ' Added to make position control possible:
  'long positionError[MAX_POSSIBLE_ENCODERS]
  'long stopDistancePresentSpeed[MAX_POSSIBLE_ENCODERS]
  'long startSpeed[MAX_POSSIBLE_ENCODERS]
  'long topSpeed[MAX_POSSIBLE_ENCODERS]
  'long distanceToTopSpeed[MAX_POSSIBLE_ENCODERS]
  'long adjustedAcceleration[MAX_POSSIBLE_ENCODERS]
  'long pseudoTargetSpeed[MAX_POSSIBLE_ENCODERS]
  'long pseudoMinTargetSpeed[MAX_POSSIBLE_ENCODERS]
  'long pseudoMaxTargetSpeed[MAX_POSSIBLE_ENCODERS]
  long targetX[MAX_POSSIBLE_ENCODERS]
  long nextLoopTimer', pseudoDecimalPoints
  'long firstDebugRow, lastDebugRow
  long activeDebugRow, lastDebugX
  long activeDebugX, firstActiveChannel, lastActiveChannel
  long ledParameter, ledParameterPtr, ledChannel
  long debugLock, childDataPtr
  long pulseBufferPtr, pulseBufferSize

  long legacyLongsNotUsed[MAX_POSSIBLE_ENCODERS]

  ''***
  long transitionTime[MAX_POSSIBLE_ENCODERS]
  ''***

  long timeOfLastPulse[MAX_POSSIBLE_ENCODERS]

  ''***
  long totalEncoderCount[MAX_POSSIBLE_ENCODERS]
  ''***
  
  long jerk[MAX_POSSIBLE_ENCODERS]
  long acceleration[MAX_POSSIBLE_ENCODERS]
  long distanceTravelled[MAX_POSSIBLE_ENCODERS]
  long distanceRemaining[MAX_POSSIBLE_ENCODERS]

  long encoderDirection[MAX_POSSIBLE_ENCODERS] 
  '' The above longs need to be kept in order and together.

VAR

  ' The variable with the "x" prefix are thosed used by
  ' the "ExperimentalSpeed" method.
  long stoppedMotorCount[Encoders#TOTAL_ENCODERS], alternatingEncoder[Encoders#TOTAL_ENCODERS]
  long alternatingTimeStamp[Encoders#TOTAL_ENCODERS]
  long alternatingDeltaTime[Encoders#TOTAL_ENCODERS]
  long previousPosition[Encoders#TOTAL_ENCODERS]
  long previousTimeStamp[Encoders#TOTAL_ENCODERS]
  long bufferIndex, positionBuffer[POSITION_BUFFER_SIZE * Encoders#TOTAL_ENCODERS]
 ' long xPositionBuffer[POSITION_BUFFER_SIZE * Encoders#TOTAL_ENCODERS]
  'long xTimeBuffer[POSITION_BUFFER_SIZE * Encoders#TOTAL_ENCODERS]
  'long xPeriodSpeed[Encoders#TOTAL_ENCODERS]
  'long xAverageSpeed[Encoders#TOTAL_ENCODERS], xBufferTotal[Encoders#TOTAL_ENCODERS]
  long deltaPosition[Encoders#TOTAL_ENCODERS], currentSpeed[Encoders#TOTAL_ENCODERS]
  'long xPeriodDistance[Encoders#TOTAL_ENCODERS], xPeriodTime[Encoders#TOTAL_ENCODERS]
  'long xPreviousCycleTime[Encoders#TOTAL_ENCODERS]
  long fullCycleTimeStamp[Encoders#TOTAL_ENCODERS]
  'long xCycleSingleDeltaTime[Encoders#TOTAL_ENCODERS]
  'long xPreviousCycleEncoder[Encoders#TOTAL_ENCODERS]
  long alternatingFullCycle[Encoders#TOTAL_ENCODERS]
  'long xCycleSingleDistance[Encoders#TOTAL_ENCODERS]
  'long xCyclePeriodDistance[Encoders#TOTAL_ENCODERS], xCyclePeriodTime[Encoders#TOTAL_ENCODERS]
  'long alternatingFullCycleBuffer[Encoders#TOTAL_ENCODERS * POSITION_BUFFER_SIZE]
  'long fullCycleTimeStampBuffer[Encoders#TOTAL_ENCODERS * POSITION_BUFFER_SIZE]
  long halfSecondDeltaPosition[Encoders#TOTAL_ENCODERS]
  long halfSecondDeltaTime[Encoders#TOTAL_ENCODERS]
  long timeBuffer[Encoders#TOTAL_ENCODERS * POSITION_BUFFER_SIZE] 
  'long xCyclePeriodSpeed[Encoders#TOTAL_ENCODERS], xCycleSingleSpeed[Encoders#TOTAL_ENCODERS] 
  long altDataPtr[Encoders#TOTAL_ENCODERS]
  long bufferedSpeed[Encoders#TOTAL_ENCODERS]

  byte activeChannelFlag[MAX_POSSIBLE_ENCODERS], stillStoppedFlag[MAX_POSSIBLE_ENCODERS]
  byte activeParameter, previousActiveParameter
  byte tempMessageFlag[TEMP_MESSAGE_TYPES], tempMessageLoc[2 * TEMP_MESSAGE_TYPES]
  byte message0[MAX_MESSAGE_SIZE], message1[MAX_MESSAGE_SIZE], message2[MAX_MESSAGE_SIZE]
  byte newSpeedFlag, motorMode[MAX_POSSIBLE_ENCODERS], lineCharacter
 
OBJ
 
  Pst : "Parallax Serial Terminal"                      ' uses one cog
  Encoders : "QuadratureMotors"                         ' uses one cog
  Header : "HeaderQuadratureMotorsArlo"
  Format : "StrFmt"    

PUB Main 
'' This method starts in cog #0
'' This method starts three additional cogs (encoder/motor, serial driver and controlLoop).
'' Presently there are four unused cogs.
  
  directionCorrection[0] := Header#FORWARD0  ' Note about direction variables
  directionCorrection[1] := Header#FORWARD1  ' in comments under "Encoder Information."

  '' "directionCorrection" allows direction reversal without requiring wires be physically moved.
  '' If the encoder direction is reversed from direction desired, change the "FORWARDx" to -1 
  '' in header object.
  '' Motor direction may be reversed by swapping "RED" and "BLACK" pin asignments in header
  '' object.

  '' Many parameters have been grouped into an array called "parentParameter".
  '' The "parentParameter" array make it easier to modify each parameter
  '' as the index of the array is the "activeParameter".
   
  acceleration := Header#PSEUDO_ACCELERATION
  
  bufferSize := MAX_ALLOWED_BUFFER

  'singlePwmResTicks := SetSinglePwmResTicks  
  {
  minSpeedTicks := TtaMethod(clkfreq , parentParameter[RESOLUTION_PARAMETER], {
                 } Header#MAX_TRANSITIONS_PER_SEC)
  }
  zeroSpeedTime := minSpeedTicks' + (minSpeedTicks / 2)

  firstActiveChannel := DEFAULT_FIRST_CHANNEL
  lastActiveChannel := DEFAULT_LAST_CHANNEL  
  'lastDebugRow := DEFAULT_LAST_ROW
  'activeDebugRow := firstDebugRow := 26 '24 '30
  debugLock := locknew

  lineCharacter := "z"

  Pst.Start(DEBUG_BAUD) '115200)

  controlCog := cognew(ControlLoop, @controlStack)
  
  waitcnt(clkfreq / 2 + cnt)

  DebugLoop(HEADING_ROWS)
  
PRI DebugLoop(location) | frozenCnt, frozenIna, combinedDirection, newTime, oldTime, {
} channelIndex, localBuffer[4], initialLocation, previousStack 

  DisplayHeading
  
  watchdogTimer := oldTime := cnt
  measuredStack := 0
  previousStack := -1
  initialLocation := location 
  repeat
    newTime := cnt
    debugInterval := newTime - oldTime
    oldTime := newTime
    
    location := initialLocation

    measuredStack := CheckStack(@controlStack, CONTROL_LOOP_STACK, previousStack)

    repeat until not lockset(debugLock)

    result := Pst.RxCount
    if result
      ReadInput

    
    'if motorMode[0] == POSITION_MODE  ' SPEED_MODE
    'location += DisplayPositions(location, firstActiveChannel, lastActiveChannel)
      'next
     
    location += DisplayResolutionPlus(location)
    'location += DividingLine(location, DIVIDING_CHARACTER, string("Active Parameter"))
    'location += DisplayActiveParameter(location)
  
    'longmove(@frozenTransitionTime, @transitionTime, LONGS_TO_FREEZE)
    '' Encoder data is copied to "frozen" arrays so the values don't change
    '' while being compared. This insures the "previous" value stored is
    '' the same value used in the comparison.

    'location += DisplayPids(location, firstActiveChannel, lastActiveChannel)
    location += DividingLine(location, DIVIDING_CHARACTER, string("Target Speeds"))
    location += DisplayTargetSpeeds(location, firstActiveChannel, lastActiveChannel)

    location += DisplayEncoderSpeeds(location, firstActiveChannel, lastActiveChannel)

    'location += DividingLine(location, DIVIDING_CHARACTER, string("Encoder Data"))

    'location += DisplayEncoderChannels(location, 2, 2)'firstActiveChannel, lastActiveChannel)

    lockclr(debugLock)
    
    'firstDebugRow := location

PRI CheckStack(localPtr, localSize, previousSize)

  localSize--
  previousSize--
  repeat result from 0 to localSize
    if long[localPtr][result]
      if result > previousSize
        previousSize := result
  result := ++previousSize
  
PUB ReadInput : localCharacter

  localCharacter := Pst.CharIn
  
  case localCharacter
    "0".."0" + Encoders#MAX_ENCODER_INDEX:
      GetSpeedInput(localCharacter)
    "c", "C":
      Pst.Clear
      Pst.Home  

PUB GetSpeedInput(channelIndex) 
'' The debug lock needs to be set prior to calling this method.         
    
  channelIndex -= "0"           ' Convert from ASCII character to number

  targetSpeed[channelIndex] := Pst.DecIn

PUB DisplayResolutionPlus(location) | pasmLoopTime

  'Pst.Position(0, location) 'RESOLUTION_PLUS_LOC_Y)
  result += NewLine
  Pst.Str(string("  PWM and Encoder loop time = "))
  pasmLoopTime := Encoders.GetDebugTime
  if pasmLoopTime > longestPasmLoop
    longestPasmLoop := pasmLoopTime
  Pst.Dec(pasmLoopTime)
  Pst.Str(string(" clock cycles or ")) 
  Pst.Dec(pasmLoopTime / MICROSECOND)
  Pst.Str(string(" us. The longest loop observed has been ")) 
  Pst.Dec(longestPasmLoop)
  Pst.Str(string(" clock cycles or ")) 
  Pst.Dec(longestPasmLoop / MICROSECOND)
  Pst.Str(string(" us.")) 
  result += NewLine
  Pst.Str(string("  The control loop cog has a disignated stack size  of "))
  Pst.Dec(CONTROL_LOOP_STACK)
  Pst.Str(string(" longs. It appears the cog has used ")) 
  Pst.Dec(measuredStack)
  Pst.Str(string(" of these longs.")) 
  result += NewLine
  Pst.Str(string("  The current resolution of the PWM driver is "))
  Pst.Dec(Encoders.GetResolution)
  Pst.Str(string(". The current frequency is ")) 
  Pst.Dec(Encoders.GetFrequency)
  Pst.Str(string(" Hz. measuredStack = ")) 
  Pst.Dec(measuredStack)
  result += NewLine
  Pst.Str(string("  The debug loop takes "))
  Pst.Dec(debugInterval)
  Pst.Str(string(" clock cycles or ")) 
  Pst.Dec(debugInterval / MILLISECOND )
  Pst.Str(string(" ms to complete. controlLoopCounter = "))
  Pst.Dec(controlLoopCounter)
  Pst.ClearEnd
  'result := RESOLUTION_PLUS_ROWS
   
PUB DisplayTargetSpeeds(location, firstChannel, lastChannel) | channelIndex

  Pst.Position(0, location) 'TARGET_SPEED_LOC_Y)
  
  repeat channelIndex from firstChannel to lastChannel 'Encoders#MAX_ENCODER_INDEX
    result += DisplayTargetSpeed(channelIndex)
    'if result <> Encoders#MAX_ENCODER_INDEX  
      'Pst.NewLine
  'DividingLine(DIVIDING_CHARACTER, 0)
  'firstChannel++
  'result := (lastChannel - firstChannel) * TARGET_SPEED_ROWS 
    
PUB DisplayTargetSpeed(channelIndex) | tempString[4]

  'NewLine
  result += NewLine
  Pst.Str(string(" targetSpeed["))
  Pst.Dec(channelIndex)
  Pst.Str(string("] ="))
  Pst.Dec(targetSpeed[channelIndex])
  'Format.fdec(@tempString, targetSpeed[channelIndex], 6, 0)
  'Pst.Str(@tempString)
  
  Pst.Str(string(", output ="))
  Pst.Dec(output[channelIndex])
  'Format.rdec(@tempString, output[channelIndex], 6)
  'Pst.Str(@tempString)

PUB DisplayEncoderSpeeds(location, firstChannel, lastChannel) | channelIndex, ptr

  repeat channelIndex from firstChannel to lastChannel
    result+= DisplayEncoderSpeed(location, channelIndex)
  
PUB DisplayEncoderSpeed(location, channelIndex)

  result += NewLine
  Pst.Str(string("stoppedMotorCount = "))
  Pst.Dec(stoppedMotorCount[channelIndex])
  
  Pst.Str(string(", alternatingEncoder = "))
  Pst.Dec(alternatingEncoder[channelIndex])
   
  result += NewLine
  Pst.Str(string("currentSpeed = "))
  Pst.Dec(currentSpeed[channelIndex])
  Pst.Str(string(", bufferedSpeed = "))
  Pst.Dec(bufferedSpeed[channelIndex])
  
  
  result += NewLine
  Pst.Str(string("rpm = "))
  DecPoint(ComputeRpm(currentSpeed[channelIndex], 1, 1000, Header#TRANS_PER_REV_NUMERATOR, {
  } Header#TRANS_PER_REV_DENOMINATOR), 1000)
  Pst.Dec(currentSpeed[channelIndex])

PUB DisplayHeading

  Pst.Home
  'NewLine
  Pst.Str(string("        name of program = "))
  Pst.Str(@programName)
  result += NewLine
  Pst.Str(string(" name of encoder object = "))
  Pst.Str(Encoders.GetObjectName)
  result += NewLine
  Pst.Str(string("  name of header object = "))
  Pst.Str(Header.GetObjectName)
  Pst.ClearEnd
  'Pst.NewLine
  DividingLine(3, DIVIDING_CHARACTER, 0)
  result += NewLine
  Pst.Str(string("  Press ", QUOTE, "c", QUOTE, " to refresh data screen."))
  Pst.ClearEnd

PUB DividingLine(location, dividingCharacter, dividingString) | localBuffer[(DIVIDING_CHARACTERS / 4) + 1]
'' The debug lock needs to be set prior to calling this method.         
  Pst.Position(0, location)
  Pst.Dec(location) 
  result := DIVIDING_CHARACTERS

  if dividingString
    result -= strsize(dividingString)
    result /= 2

  Format.chstr(@localBuffer, dividingCharacter, result)
  Pst.str(@localBuffer)
  if dividingString
    Pst.str(dividingString)
    Pst.str(@localBuffer)
  Pst.ClearEnd
 
  result := DIVIDING_ROWS

PUB ComputeRpm(ticksPerHalfSecond, direction, scale, numerator, denominator)

  result := ticksPerHalfSecond * 120 * denominator * direction / numerator
  result *= scale

PUB TtaMethodSigned(N, X, D)   ' return X*N/D where all numbers and result are positive =<2^31

  result := 1
  if N < 0
    -N
    -result
  if X < 0
    -X
    -result
  if D < 0
    -D
    -result
    
  result *= TtaMethod(N, X, D)

PUB TtaMethod(N, X, D)   ' return X*N/D where all numbers and result are positive =<2^31
  return (N / D * X) + (binNormal(N//D, D, 31) ** (X*2))

PUB BinNormal (y, x, b) : f                  ' calculate f = y/x * 2^b
' b is number of bits
' enter with y,x: {x > y, x < 2^31, y <= 2^31}
' exit with f: f/(2^b) =<  y/x =< (f+1) / (2^b)
' that is, f / 2^b is the closest appoximation to the original fraction for that b.
  repeat b
    y <<= 1
    f <<= 1
    if y => x    '
      y -= x
      f++
  if y << 1 => x    ' Round off. In some cases better without.
      f++
                       
PUB ControlLoop | channelIndex

  Encoders.StartEncoders(Header.GetEncoderPins, @totalEncoderCount, @transitionTime, Header.GetEnablePins, {
  } Header.GetRedPins, Header.GetBlackPins, @output, @encoderDirection)

  pulseBufferPtr := Encoders.GetBufferLocation
  pulseBufferSize := Encoders.GetBufferSize

  controlAlgorithmInterval :=  Header#REFRESH_RATE * MILLISECOND

  nextLoopTimer := cnt + controlAlgorithmInterval
  
  repeat

    waitcnt(nextLoopTimer += controlAlgorithmInterval)
    
    ComputeEncoderSpeeds(firstActiveChannel, lastActiveChannel)
    
    RampSpeeds(firstActiveChannel, lastActiveChannel)

    ComputeOutputs(firstActiveChannel, lastActiveChannel)

    result := Encoders.RefreshPower ' The value of "output" will be used by the
    ' child object to set the power to the motors.

    controlLoopCounter++

PUB RampSpeeds(firstChannel, lastChannel) : channelIndex

  repeat channelIndex from firstChannel to lastChannel
    RampSpeed(channelIndex)

PUB RampSpeed(channelIndex) | pseudoTarget, pseudoLocalAccel

  pseudoTarget := targetSpeed[channelIndex] * Header#PSEUDOREAL_MULTIPLIER
  pseudoLocalAccel := -acceleration #> pseudoTarget - pseudoRampedSpeed[channelIndex] {
  } <# acceleration
  
  pseudoRampedSpeed[channelIndex] += pseudoLocalAccel

PUB ComputeOutputs(firstChannel, lastChannel) : channelIndex| localSpeed

  repeat channelIndex from firstChannel to lastChannel
    output[channelIndex] := pseudoRampedSpeed[channelIndex] / Header#PSEUDOREAL_MULTIPLIER
  
PUB ComputeEncoderSpeeds(firstChannel, lastChannel) : channelIndex

  repeat channelIndex from firstChannel to lastChannel
    ComputeEncoderSpeed(channelIndex)
  bufferIndex++
  bufferIndex //= POSITION_BUFFER_SIZE
  
PRI ComputeEncoderSpeed(channelIndex) '| adjustmentBits, adjustedTime 
'' A "Period" is a half second amount of time.
'' A "Cycle" is a full encoder cycle
'' Todo count times a full cycle isn't received.
'' Switch from full cycle monitoring to single tick
'' monitoring at very low speeds.
'' The data produced by this method is presently not
'' used by the program.

  'adjustmentBits := INITIAL_BIT_ADJUSTMENT
  
  altDataPtr[channelIndex] := Encoders.GetPointer(channelIndex)
  {The act of requesting the pointer changes the address where
  future writes will occur. This keeps the data located at
  the returned pointer from being overwritten by the encoder
  reading cog.
  Data from the buffer may now be read without concern of it
  being overwritten.
  Both the last transition count with time and last full cycle
  count with time are written to the buffer.
  Encoder which aren't precisely 90 degrees out of phase
  should benifit from being monitored by full cycles.}
  
  alternatingEncoder[channelIndex] := long[altDataPtr[channelIndex]]
  alternatingTimeStamp[channelIndex] := long[altDataPtr[channelIndex] + 4]
  alternatingFullCycle[channelIndex] := long[altDataPtr[channelIndex] + 8]
  fullCycleTimeStamp[channelIndex] := long[altDataPtr[channelIndex] + 12]
  
  alternatingDeltaTime[channelIndex] := alternatingTimeStamp[channelIndex] - previousTimeStamp[channelIndex]
  deltaPosition[channelIndex] := alternatingEncoder[channelIndex] - previousPosition[channelIndex]

  if deltaPosition[channelIndex] and output[channelIndex]
    stoppedMotorCount[channelIndex] := 0
    currentSpeed[channelIndex] := ComputeSpeed(alternatingDeltaTime[channelIndex], {
    } deltaPosition[channelIndex])
  else
    stoppedMotorCount[channelIndex]++
    currentSpeed[channelIndex] := 0

  halfSecondDeltaPosition[channelIndex] := alternatingEncoder[channelIndex] - {
  } positionBuffer[POSITION_BUFFER_SIZE * channelIndex + bufferIndex]
  positionBuffer[POSITION_BUFFER_SIZE * channelIndex + bufferIndex] := alternatingEncoder[channelIndex]
   
  halfSecondDeltaTime[channelIndex] := alternatingTimeStamp[channelIndex] - {
  } timeBuffer[POSITION_BUFFER_SIZE * channelIndex + bufferIndex]
  timeBuffer[POSITION_BUFFER_SIZE * channelIndex + bufferIndex] := alternatingTimeStamp[channelIndex]
  
  
  ' The value "10" reduces the values sent to "ComputeSpeed" method to manageable
  ' amounts. This is a hack and should be improved.
  bufferedSpeed[channelIndex] := ComputeSpeed(halfSecondDeltaTime[channelIndex] / 10, {
    } halfSecondDeltaPosition[channelIndex] / 10)
    
  previousTimeStamp[channelIndex] := alternatingTimeStamp[channelIndex]
  previousPosition[channelIndex] := alternatingEncoder[channelIndex]

PRI ComputeSpeed(deltaTime, localDeltaPosition) 

  result := TtaMethodSigned(SPEED_NUMERATOR, localDeltaPosition, deltaTime * {
  } ADJ_TO_TICKS_PER_HALF_SECOND)
        
PUB NewLine

  Pst.ClearEnd
  Pst.Newline
  result := 1
  
PUB DecPoint(value, denominator)
  if value < 0
    Pst.Char("-")
    -value
      
  if value => denominator
    result := value / denominator
    Pst.Dec(result)
    value //= denominator     
  else    
    Pst.Char("0")
  Pst.Char(".")  
  repeat while denominator > 1
    denominator /= 10
    if value => denominator
      result := value / denominator
      Pst.Dec(result)
      value //= denominator
    else
      Pst.Char("0")
      