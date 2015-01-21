DAT objectName          byte "QuadratureMotors", 0
CON
{{
  ******* Public Notes *******

  This program is used to monitor four quadrature
  encoders like those used by the Rover 5 chassis.
  See comments at start of each method.

  This object was written by Duane Degn

  The code based on a the encoder technique shown in a
  Nuts & Volts article by Jon "JonnyMac" McPhalen.
  
}}
{
  ******* Private Notes *******
  
  FourQuadratureEncoders130125a modifed from VexMecanumRc121219b
  130125a Start removing non-encoder code from program and
  make this object a child object.
  0205a Add additional comments in preperation to posting to forum.
  15a Change to pointer to pins so pairs can be in any order.
  0314a Start adding ability to time pulses.
  131126a Add motor control masks.
  27d Kind of works with 27d parent and 27c grandparent.
  28a Kind of works with 28a parent and 28b grandparent.
  Change name from "FourQuadratureEncoders131128c" to
  "FourQuadratureMotors131128a".
  28b Abandon 28a. I changed too many things at once.
  1207a Try adding servo control.
  11c Try to move readings of speed to end of PWM period.
  11d Remove some unused variables and buffer.
  13a Start to add comments on how to convert to two motors/encoders.
  13b Added comments and moved variable postions.
  17a Add ability to adust averaging buffer.
  Change name from "FourQuadratureMotors140429a" to "XQuadratureMotors150116a"
  150117a Start adding self modifying code.
  17a So far so good.
  17b Still works.
  17c I broke it.
  17d Works again. I forgot to use # in a couple of places.
  17e Changed more PWM code. Still works.
  17g Does not work.
  17h Works but very little has changed from 17f.
  17i A few more changes. Still works.
  17j Still works.
  17k Still appears to work. I can control PWM and monitor encoders but I'm not
  sure if speed sections are working.
  17l Still works.
  17m Does not work.
  17n Revert to 17l. Change different section of code than changed with 17m.
  17n Appears to work correctly.
  17o Still works.
  17p Does not work. The motor will start and then very quickly stop.
  The encoder (on channel 2) alternates between zero and one.
  17q Revert to 17o. My attempt (in 17p) to eliminate the variable "pulsePtr"
  failed. This time I'll try to eliminate "pulseHead".
  17q Apparently still working.
  17r Does not work. Crahes parent object.
  17s Revert to 17q.
  17s Eliminated "directionFlagPtr". Still works
  17t Doesn't work. No encoder feedback and no PWM control
  17u Revert to 17s. Small change in the order of the code. Still works.
  17v Appears to work! Nice and consoludated.
  18a Clean the codeup a bit. Lots of changes to include latest encoder
  count in an alternating buffer. Probably too many changes.
  18a The program still appears to work.
  18c Doesn't appear to work.
  18d Revert to 18a.
  18d Still doesn't read encoders properly.
  18e Revert to 17w.
  18e Encoder works.
  18g Motor and encoder dirctions changed.
  18h Still works. First element written to new buffer. The total encoder count is now
  written twice.
  
                    
}
CON

  TOTAL_ENCODERS = 2            '' User changeable
                                '' It is possible to change to any number (within reason)
                                '' of motor with encoders
                            
                                                               
  MAX_ENCODER_INDEX = TOTAL_ENCODERS - 1                              
  'SPEED_BUFFER_SIZE = 16
  
  DEFAULT_PWM_FREQUENCY = 200         '' user changeable
  DEFALUT_PWM_RESOLUTION = 1_000     '' user changeable

  MAX_BITS_TO_AVERAGE = 7      '' user changeable
  MIN_BITS_TO_AVERAGE = 1      '' user changeable

  DEFAULT_BITS_TO_AVERAGE = 5   '' user changeable
 
  MAX_BUFFER_SIZE = 1 << MAX_BITS_TO_AVERAGE '|< (MAX_BITS_TO_AVERAGE - 1)
 
  'MAX_BUFFER_INDEX = MAX_BITS_TO_AVERAGE - 1

  SERVO_FREQUENCY = 50
  SERVO_STOP = 1500
  SERVO_FULL_SPEED = 500          '' user changeable

  BYTES_PER_MINI_BUFF = 16
    
VAR

  'long speedBuffer[SPEED_BUFFER_SIZE * TOTAL_ENCODERS]
  long cog
  long debugTime
  long pulseBuffer[MAX_BUFFER_SIZE * TOTAL_ENCODERS]
  long pwmOnTime[TOTAL_ENCODERS], pwmTime
  long pwmFrequency, pwmResolution
  long targetPtr, negativePinPtr, positivePinPtr
  long largestMotorsInUseIndex
  long servoStopTics, servoFullSpeedTics
  long negPwmResolution, encoderPtr, encoderPinPtr
  long speedPtr, enablePinPtr
  long bufferSize', bufferBitsToShift', maxBufferIndex
  long dataPairAddress
  long dataPairBufferA[4 * TOTAL_ENCODERS]
  long dataPairBufferB[4 * TOTAL_ENCODERS]
  
  byte servoFlag
            
PUB StartEncoders(encoderPinPtr_, encoderPtr_, speedPtr_, enablePinPtr_, positivePinPtr_, {
  } negativePinPtr_, targetPtr_, directionFlagPtr_) | localIndex
'' This method expects encoder pins to be stored sequentially
'' in an array of bytes at encoderPinPtr.
'' The value "encoderPtr" should be the memory location
'' of four sequential longs where incremented encoder values
'' will be written.
'' The value "speedPtr" should be the memory location
'' of four sequential longs where the sum of "speedBuffer" values
'' will be written.
'' This method needs to be called to start using object.
'' There isn't a "Stop" method so multiple calls
'' will launch multiple cogs which will likely cause serious
'' problems for parent program.

  servoStopTics := (clkfreq / 1_000_000) * SERVO_STOP
  servoFullSpeedTics := (clkfreq / 1_000_000) * SERVO_FULL_SPEED
  encoderPinPtr := encoderPinPtr_   ' ** think about using longmove 
  encoderPtr := encoderPtr_
  speedPtr := speedPtr_
  enablePinPtr := enablePinPtr_
  'negativePinPtr := negativePinPtr_
  'positivePinPtr := positivePinPtr_
  negativePinPtr := positivePinPtr_
  positivePinPtr := negativePinPtr_
  targetPtr := targetPtr_
  dataPairAddress := @dataPairBufferA
  
  dataPairAddressPtr := @dataPairAddress


  repeat localIndex from 0 to MAX_ENCODER_INDEX
    encoderPin0[localIndex] := byte[encoderPinPtr][localIndex]
    directionFlagPtr0[localIndex] := directionFlagPtr_ + (4 * localIndex)
    pulseBufferSumPtr0[localIndex] := speedPtr + (4 * localIndex) 
    lastUpdatedPtr0[localIndex] := speedPtr + (4 * TOTAL_ENCODERS) + (4 * localIndex)
    onTimePtr0[localIndex] := @pwmOnTime + (4 * localIndex)
 
  timeAddress := @debugTime
  'buffer0 := @speedBuffer       ' first location in each buffer (static)
  'buffer1 := buffer0 + (SPEED_BUFFER_SIZE * 4)
  'buffer2 := buffer1 + (SPEED_BUFFER_SIZE * 4)
  'buffer3 := buffer2 + (SPEED_BUFFER_SIZE * 4)

  
  SetBufferSize(DEFAULT_BITS_TO_AVERAGE)
  
  
  
 { buffer0Head := buffer0        ' current location in each buffer (dynamic)
  buffer1Head := buffer1
  buffer2Head := buffer2
  buffer3Head := buffer3    
 
  pulseHead0 := 0'pulse0
  pulseHead1 := 0'pulse1
  pulseHead2 := 0'pulse2
  pulseHead3 := 0'pulse3    }
   

 { pulseBufferSumPtr0 := speedPtr 
  pulseBufferSumPtr1 := pulseBufferSumPtr0 + 4
  pulseBufferSumPtr2 := pulseBufferSumPtr1 + 4  '' >>4to2<< comment out line
  pulseBufferSumPtr3 := pulseBufferSumPtr2 + 4  '' >>4to2<< comment out line
  lastUpdatedPtr0 := pulseBufferSumPtr3 + 4   '' >>4to2<< comment out line
  'lastUpdatedPtr0 := pulseBufferSumPtr1 + 4 '' >>4to2<< uncomment line
  lastUpdatedPtr1 := lastUpdatedPtr0 + 4      
  lastUpdatedPtr2 := lastUpdatedPtr1 + 4       '' >>4to2<< comment out line
  lastUpdatedPtr3 := lastUpdatedPtr2 + 4       '' >>4to2<< comment out line
    }
  repeat localIndex from 0 to MAX_ENCODER_INDEX 
    result := long[enablePinPtr][localIndex]
    if result == -1
      next
    largestMotorsInUseIndex := localIndex  
    enableMask0[localIndex] := 1 << result
    result := long[positivePinPtr][localIndex]
    if result == -1
      servoFlag := 1
      next
    dira[result] := 1
    if result <> -1
      dira[long[negativePinPtr][localIndex]] := 1

    
  SetFrequency(DEFAULT_PWM_FREQUENCY)
  SetResolution(DEFALUT_PWM_RESOLUTION)
  
  {onTimePtr0 := @pwmOnTime
  onTimePtr1 := onTimePtr0 + 4
  onTimePtr2 := onTimePtr1 + 4   '' >>4to2<< comment out line
  onTimePtr3 := onTimePtr2 + 4   '' >>4to2<< comment out line
  }
  pwmTimePtr := @pwmTime
  pwmTimeCog := pwmTime

  Stop
  
  result := cog := cognew(@enter, encoderPtr) + 1

PUB Stop
'' frees a cog

  if cog
    cogstop(cog~ - 1)
  
PUB GetPointer(channel)
'' Returns address of active buffer for the desired channel.
'' The active buffer is changed to the alternate
'' address to keep the data pair consistent.

  result := dataPairAddress + (channel * BYTES_PER_MINI_BUFF)
  if dataPairAddress == @dataPairBufferA
    dataPairAddress := @dataPairBufferB
  else
    dataPairAddress := @dataPairBufferA
    
PUB GetPairPtr

  result := @dataPairBufferA
  
PUB SetBitsToAverage(localBits)

  if cog
    Stop
    result := StartEncoders(encoderPinPtr, encoderPtr, speedPtr, enablePinPtr, positivePinPtr, {
    } negativePinPtr, targetPtr, directionFlagPtr0)
     
PRI SetBufferSize(localBits) | localIndex

  bufferBitsToShift := MIN_BITS_TO_AVERAGE #> localBits {
                          } <# MAX_BITS_TO_AVERAGE
  'bufferBitsToShift -= 1 
  bufferSize := 1 << bufferBitsToShift '|< bufferBitsToShift
  headRollover := bufferSize * 4
  repeat localIndex from 0 to MAX_ENCODER_INDEX
    pulsePtr0[localIndex] := @pulseBuffer + (localIndex * headRollover)
   
PUB SetFrequency(localFrequency)

  if servoFlag
    pwmFrequency := SERVO_FREQUENCY
  else
    pwmFrequency := localFrequency
  pwmTime := clkfreq / pwmFrequency
  
PUB SetResolution(localResolution)

  pwmResolution := localResolution
  negPwmResolution := -1 * pwmResolution
  
PUB RefreshSpeed | localSpeed, localIndex

  result := @pwmOnTime

  if servoFlag
    repeat localIndex from 0 to largestMotorsInUseIndex
      localSpeed := long[targetPtr][localIndex]
      localSpeed <#= pwmResolution
      localSpeed #>= negPwmResolution
      pwmOnTime[localIndex] := servoStopTics + ((servoFullSpeedTics * localSpeed) / pwmResolution) 
  else
    repeat localIndex from 0 to largestMotorsInUseIndex
      localSpeed := long[targetPtr][localIndex]
      if localSpeed == 0
        outa[long[positivePinPtr][localIndex]] := 0
        if outa[long[negativePinPtr][localIndex]] <> -1
          outa[long[negativePinPtr][localIndex]] := 0
        pwmOnTime[localIndex] := 0
        next  
      elseif localSpeed > 0
        if outa[long[negativePinPtr][localIndex]] <> -1
          outa[long[negativePinPtr][localIndex]] := 0
        outa[long[positivePinPtr][localIndex]] := 1
      else
        -localSpeed
        outa[long[positivePinPtr][localIndex]] := 0
        if outa[long[negativePinPtr][localIndex]] <> -1
          outa[long[negativePinPtr][localIndex]] := 1  
      localSpeed <#= pwmResolution
      pwmOnTime[localIndex] := (pwmTime * localSpeed) / pwmResolution
 
PUB GetBufferSize

  result := bufferSize
  
PUB GetResolution

  result := pwmResolution
  
PUB GetFrequency

  result := pwmFrequency
  
PUB GetObjectName
'' This method returns the name of this object.
'' This method is not needed for proper operation of object
'' and may be deleted.

  result := @objectName

PUB GetDebugTime
'' This method returns the value of "debugTime" to aid
'' in debugging program.
'' The value of "debugTime" is the number of clock cycles
'' the last PASM loop took to complete.
'' This value may be used to determine the practical limits
'' on frequency and resolution settings.
'' This method is not needed for proper operation of object
'' and may be deleted.

  result := debugTime
  
PUB GetBufferLocation
'' This method returns the location of "speedBuffer" to aid
'' in debugging program.
'' This method is not needed for proper operation of object
'' and may be deleted.

  result := @pulseBuffer
  
DAT                     org       0

enter                   mov     encoderAddress, par 
                        mov     encoderAddress0, encoderAddress 
                       
                        mov     oldscan, ina                   ' get start-up inputs
                        mov     channelCountdown, encodersInUse
moveToNextOldScan       mov     oldscan0, oldscan
shiftPins               shr     oldscan0, encoderPin0
andBits                 and     oldscan0, #%11
movToNextAddress        mov     encoderAddress1, encoderAddress0
addToNextAddress        add     encoderAddress1, #4
orEnableMask            or      dira, enableMask0
                        add     moveToNextOldScan, destinationIncrement
                        add     shiftPins, destAndSourceIncrement
                        add     andBits, destinationIncrement
                        add     movToNextAddress, destAndSourceIncrement
                        add     addToNextAddress, destinationIncrement
                        add     orEnableMask, sourceIncrement
                        djnz    channelCountdown, #moveToNextOldScan
                     
                        mov     timer, cnt                      ' sync with system cnt
                        mov     pwmTimer, timer
                        'add     timer, enctiming                ' set loop timing                        
                        mov     speedTimer, timer

'-------------------------------------------------------------------------------

encloop                 mov     nowTime, cnt    ' use the same time point for all comparisons
                                         ' if source is larger, write c

                        mov     pwmTimeTemp, nowTime 
                        subs    pwmTimeTemp, pwmTimer
                        movd    compareOnTimes, #onTimeCog0
                        movs    turnOff, #enableMask0
                        mov     channelCountdown, encodersInUse
compareOnTimes          cmps    0-0, pwmTimeTemp wc ' Check to see if channel should be                      
turnOff       if_c      andn    outa, 0-0          ' turned off.
                        add     compareOnTimes, destinationIncrement
                        add     turnOff, sourceIncrement
                     
                        djnz    channelCountdown, #compareOnTimes
                        cmps    pwmTimeCog, pwmTimeTemp wc 
              if_c      jmp    #checkPwm        ' PWM period is over
                        
              
continueLoop            mov     previousTime, loopTimer   ' first reading will be garbage
                        mov     loopTimer, cnt            ' find time to execute loop
                        mov     deltaTime, loopTimer
                        sub     deltaTime, previousTime
                                   
                        wrlong  deltaTime, timeAddress             

                        mov     originalScan, ina               ' get current inputs

                        movs    shiftNewScanRight, #encoderPin0
                        movs    setOldScanValue, #oldscan0
                        movd    movToOldScan, #oldscan0
                        movd    addChange, #encoderTotal0
                        movd    writeTotal, #encoderTotal0
                        movs    writeTotal, #encoderAddress0
                        movs    subtractLastPulse, #lastPulse0
                        movd    newPulseBecomesLast, #lastPulse0
                        movs    writePulseBufferSum, #pulseBufferSumPtr0
                        movs    addPulseHead, #pulseHead0
                        {movd    addPulseHead, #pulsePtr0
                        movs    readFromPulsePtr, #pulsePtr0
                        movs    writeToPlusPtr, #pulsePtr0 }
                        movd    addFourToPulseHead, #pulseHead0
                        movd    checkForRollover, #pulseHead0  
                        movs    writeLastUpdateTime, #lastUpdatedPtr0   
                        movd    subtractOldPulse, #pulseTotal0
                        movd    addNewtime, #pulseTotal0
                        movs    movPulseTotalTempTotal, #pulseTotal0 
                        movs    writeToDirectionFlagPtr, #directionFlagPtr0 
                        movs    setPulsePtr, #pulsePtr0
                        movd    addChangeToQuanta, #encoderQuanta0

                        rdlong  encoderAddressTemp, dataPairAddressPtr

                        movd    recordTimeOfTransition, #lastTransition0
                        movd    updateTransitionTime, #lastTransition0
                        movd    recordTimeOfCycle, #lastEncoderCycle0
                        movd    fullCycleCount, #fullCycleCount0
                        movs    fullCycleCount, #encoderTotal0
                        movd    writeEncoderTotal, #encoderTotal0
                        movd    updateFullCycleCount, #fullCycleCount0
                        movd    updateFullCycleTime, #lastEncoderCycle0 
                        mov     channelCountdown, encodersInUse
                        
                        mov     newCycleFlag, zero
                        
originalScanValues      mov     newscan, originalScan           
shiftNewScanRight       shr     newscan, 0-0 'encoderPin0
setOldScanValue         mov     oldScan, 0-0 'oldscan0
           
                       { mov     newscan, originalScan           
                        shr     newscan, encoderPin0
                        mov     oldScan, oldscan0  }             
                        'mov     encoderTotal, encoderTotal0     
                        'mov     encoderAddress, encoderAddress0
                                
                        'mov     lastPulse, lastPulse0
                        'mov     pulseBufferSumPtr, pulseBufferSumPtr0  ' no change
                        'mov     pulseHead, pulseHead0   ' will change
                        'mov     lastUpdatedPtr, lastUpdatedPtr0    ' no change
                        'mov     pulseTotal, pulseTotal0   ' will change
setPulsePtr             mov     pulsePtr, 0-0 'pulsePtr0       ' no change
'                        mov     pulsePtr, pulsePtr0       ' no change
                        'mov     directionFlagPtr, directionFlagPtr0

                        call    #chkEncoder                     ' run algorithm with active variables

movToOldScan            mov     0-0, newscan 'oldscan0, newscan                        
                        'mov     oldscan0, newscan 
                        'mov     encoderTotal0, encoderTotal
addChangeToQuanta       add     0-0, encoderChange 'encoderQuanta0, encoderChange
'                        add     encoderQuanta0, encoderChange

                        '**
                        'mov     pulseTotal0, pulseTotal
                        'mov     pulseHead0, pulseHead
                        'mov     lastPulse0, lastPulse
                                                                                                
                        
                        add     shiftNewScanRight, sourceIncrement
                        add     setOldScanValue, sourceIncrement
                        add     addChange, destinationIncrement
                        add     writeTotal, destAndSourceIncrement
                        add     subtractLastPulse, sourceIncrement
                        add     newPulseBecomesLast, destinationIncrement
                        add     writePulseBufferSum, sourceIncrement

                        add     addPulseHead, sourceIncrement 'destAndSourceIncrement
                        add     addFourToPulseHead, destinationIncrement
                        add     checkForRollover, destinationIncrement

                        add     writeLastUpdateTime, sourceIncrement
                        add     subtractOldPulse, destinationIncrement
                        add     addNewtime, destinationIncrement
                        add     movPulseTotalTempTotal, sourceIncrement
                        add     writeToDirectionFlagPtr, sourceIncrement         
                        add     setPulsePtr, sourceIncrement
                        add     movToOldScan, destinationIncrement
                        add     addChangeToQuanta, destinationIncrement

                        
                        djnz    channelCountdown, #originalScanValues

                        'mov     encoderTotal, encoderTotal1     
                        'mov     encoderAddress, encoderAddress1   

                        'mov     lastPulse, lastPulse1
                        'mov     pulseBufferSumPtr, pulseBufferSumPtr1  ' no change
                        'mov     pulseHead, pulseHead1   ' will change
                        'mov     lastUpdatedPtr, lastUpdatedPtr1    ' no change
                        'mov     pulseTotal, pulseTotal1   ' will change
                       {{ mov     newscan, originalScan           
                        shr     newscan, encoderPin1
                        mov     oldScan, oldscan1

                        mov     pulsePtr, pulsePtr1       ' no change
                        'mov     directionFlagPtr, directionFlagPtr1
                        
                        call    #chkEncoder               ' run algorithm with active variables
                        mov     oldscan1, newscan 
                        'mov     encoderTotal1, encoderTotal
                        add     encoderQuanta1, encoderChange

                        'mov     pulseTotal1, pulseTotal
                        'mov     pulseHead1, pulseHead
                        'mov     lastPulse1, lastPulse

                        '' >>4to2<< comment out below
                        mov     newscan, originalScan           
                        shr     newscan, encoderPin2
                        mov     oldScan, oldscan2               
                        'mov     encoderTotal, encoderTotal2     
                        'mov     encoderAddress, encoderAddress2   
                        add     addChange, destinationIncrement
                        add     writeTotal, destAndSourceIncrement
                        add     subtractLastPulse, sourceIncrement
                        add     newPulseBecomesLast, destinationIncrement
                        add     writePulseBufferSum, sourceIncrement
                        
                        add     addPulseHead, sourceIncrement 'destAndSourceIncrement
                        add     addFourToPulseHead, destinationIncrement
                        add     checkForRollover, destinationIncrement

                        add     writeLastUpdateTime, sourceIncrement
                        add     subtractOldPulse, destinationIncrement
                        add     addNewtime, destinationIncrement
                        add     movPulseTotalTempTotal, sourceIncrement
                        add     writeToDirectionFlagPtr, sourceIncrement
                        
                        'mov     lastPulse, lastPulse2
                        'mov     pulseBufferSumPtr, pulseBufferSumPtr2  ' no change
                        'mov     pulseHead, pulseHead2   ' will change
                        'mov     lastUpdatedPtr, lastUpdatedPtr2    ' no change
                        'mov     pulseTotal, pulseTotal2   ' will change
                        mov     pulsePtr, pulsePtr2       ' no change
                        'mov     directionFlagPtr, directionFlagPtr2
                        
                        call    #chkEncoder                     ' run algorithm with active variables
                        mov     oldscan2, newscan 
                        'mov     encoderTotal2, encoderTotal
                        add     encoderQuanta2, encoderChange

                        'mov     pulseTotal2, pulseTotal
                        'mov     pulseHead2, pulseHead
                        'mov     lastPulse2, lastPulse
                        
                        mov     newscan, originalScan           
                        shr     newscan, encoderPin3
                        mov     oldScan, oldscan3               
                        'mov     encoderTotal, encoderTotal3     
                        'mov     encoderAddress, encoderAddress3   
                        add     addChange, destinationIncrement
                        add     writeTotal, destAndSourceIncrement
                        add     subtractLastPulse, sourceIncrement
                        add     newPulseBecomesLast, destinationIncrement
                        add     writePulseBufferSum, sourceIncrement
                        
                        add     addPulseHead, sourceIncrement 'destAndSourceIncrement
                        add     addFourToPulseHead, destinationIncrement
                        add     checkForRollover, destinationIncrement

                        add     writeLastUpdateTime, sourceIncrement
                        add     subtractOldPulse, destinationIncrement
                        add     addNewtime, destinationIncrement
                        add     movPulseTotalTempTotal, sourceIncrement
                        add     writeToDirectionFlagPtr, sourceIncrement
                        
                        'mov     lastPulse, lastPulse3
                        'mov     pulseBufferSumPtr, pulseBufferSumPtr3  ' no change
                        'mov     pulseHead, pulseHead3   ' will change
                        'mov     lastUpdatedPtr, lastUpdatedPtr3    ' no change
                        'mov     pulseTotal, pulseTotal3   ' will change
                        mov     pulsePtr, pulsePtr3       ' no change
                        'mov     directionFlagPtr, directionFlagPtr3
                        
                        call    #chkEncoder                     ' run algorithm with active variables
                        mov     oldscan3, newscan 
                        'mov     encoderTotal3, encoderTotal
                        add     encoderQuanta3, encoderChange  }}

                        'mov     pulseTotal3, pulseTotal
                        'mov     pulseHead3, pulseHead
                        'mov     lastPulse3, lastPulse
                        '' >>4to2<< comment out above
                        
                        jmp     #encloop

'-------------------------------------------------------------------------------
                        
DAT chkencoder          mov     encoderChange, zero
                        and     newscan, #%11                   ' about 30 instructions
                        cmp     newscan, oldScan        wz      ' check for change
              if_z      jmp     #updateAlternatingBuffer                        
              'if_z      jmp     #chkencoder_ret 'encloop 'chkencoder ' ' wait if none

                        'there is a change
                        
addPulseHead            add     pulsePtr, 0-0
'addPulseHead            add     0-0, 0-0 'pulsePtr, pulseHead
'                        add     pulsePtr, pulseHead
                        mov     newPulse, cnt
                        rdlong  oldPulse, pulsePtr
subtractOldPulse        sub     0-0, oldPulse
'                        sub     pulseTotal, oldPulse
writeLastUpdateTime     wrlong  newPulse, 0-0 'lastUpdatedPtr
'                        wrlong  newPulse, lastUpdatedPtr
                        'sub     lastPulse, newPulse  ' lastPulse is differnence between loop's cnt values
                        mov     temp, newPulse ' store this loop's cnt value in lastPulse

subtractLastPulse       sub     temp, 0-0 'lastPulse
newPulseBecomesLast     mov     0-0, newPulse
'                        sub     temp, lastPulse
'                        mov     lastPulse, newPulse
                        'shr     temp, #4
addNewtime              add     0-0, temp 'pulseTotal, temp
'                        add     pulseTotal, temp
                        
                        'wrlong  pulseTotal, pulseBufferSumPtr   ' yes stomping
movPulseTotalTempTotal  mov     tempTotal, 0-0 'pulseTotal
'                        mov     tempTotal, pulseTotal
                        shr     tempTotal, bufferBitsToShift
                        'wrlong  tempTotal, pulseBufferSumPtr   ' yes stomping
writePulseBufferSum     wrlong  tempTotal, 0-0 'pulseBufferSumPtr                ' yes stomping                         
                                     
addFourToPulseHead      add     0-0, #4 'pulseHead, #4
checkForRollover        cmpsub  0-0, headRollover 'pulseHead, headRollover               
'                        add     pulseHead, #4
'                        cmpsub  pulseHead, headRollover                
                        
                        wrlong  temp, pulsePtr                  ' not stomping
                        
                        
case11                  cmp     oldscan, #%11           wz      ' oldscan == %11?         
              if_nz     jmp     #case01                         '20 if no, try next         
                        cmp     newscan, #%01           wz      ' clockwise (postive)? Z = yes 
                        jmp     #update

case01                  cmp     oldscan, #%01           wz
              if_nz     jmp     #case00
                        cmp     newscan, #%00           wz   '40
              if_z      mov     newCycleFlag, one
                        jmp     #update                        

case00                  cmp     oldscan, #%00           wz      
              if_nz     jmp     #case10                         
                        cmp     newscan, #%10           wz      
                        jmp     #update                      '60
                                                                          
case10                  cmp     oldscan, #%10           wz
              if_nz     jmp     #chkencoder_ret                 ' (should never happen)
                        cmp     newscan, #%11           wz
              if_nz     mov     newCycleFlag, one
              
update        if_z      mov     encoderChange, negOne             ' increment value

decvalue      if_nz     mov     encoderChange, one              ' decrement
'                        wrlong  encoderChange, directionFlagPtr
writeToDirectionFlagPtr wrlong  encoderChange, 0-0 'directionFlagPtr
                        'add     encoderTotal, encoderChange                        
addChange               add     0-0, encoderChange 'encoderTotal, encoderChange                        

                        tjz     newCycleFlag, #writeTotal
recordTimeOfCycle       mov     0-0, loopTimer
fullCycleCount          mov     0-0, 0-0 'encoderTotal
                        
writeTotal              wrlong  0-0, 0-0 'encoderTotal, encoderAddress 88 to 92update hub  ' keep this
'writeTotal              wrlong  encoderTotal, encoderAddress' 88 to 92update hub  ' keep this
recordTimeOfTransition  mov     0-0, loopTimer
updateAlternatingBuffer add     recordTimeOfTransition, destinationIncrement

writeEncoderTotal       wrlong  0-0, encoderAddressTemp 'encoderTotal, encoderAddressTemp
'                        wrlong  encoderTotal, encoderAddressTemp     
                        add     recordTimeOfCycle, destinationIncrement
                        add     encoderAddressTemp, #4

updateTransitionTime    wrlong  0-0, encoderAddressTemp
                        add     encoderAddressTemp, #4                         
                        add     updateTransitionTime, destinationIncrement

updateFullCycleCount    wrlong  0-0, encoderAddressTemp
                        add     encoderAddressTemp, #4                         
                        add     updateFullCycleCount, destinationIncrement

updateFullCycleTime     wrlong  0-0, encoderAddressTemp
                        add     encoderAddressTemp, #4                         
                        add     updateFullCycleTime, destinationIncrement
                                                                   
                        add     fullCycleCount, destAndSourceIncrement
                        add     writeEncoderTotal, destinationIncrement                                               
chkencoder_ret          ret

'-------------------------------------------------------------------------------
DAT checkPwm            rdlong  pwmTimeCog, pwmTimePtr   ' 50ns * ((4 * 4) + 5)
                        add     pwmTimer, pwmTimeCog
                        
                        movd    readOnTime, #onTimeCog0
                        movs    readOnTime, #onTimePtr0
                        movd    testForZeroTime, #onTimeCog0
                        movs    turnOnEnableMask, #enableMask0
                        
                        mov     channelCountdown, encodersInUse
                        
readOnTime              rdlong  0-0, 0-0            
testForZeroTime         tjz     0-0, #afterTurnOn                       
turnOnEnableMask        or      outa, 0-0
afterTurnOn             add     readOnTime, destAndSourceIncrement
                        add     testForZeroTime, destinationIncrement
                        add     turnOnEnableMask, sourceIncrement

                        djnz    channelCountdown, #readOnTime
                        
{checkPwm1               rdlong  onTimeCog1, onTimePtr1
                        tjz     onTimeCog1, #checkPwm2     '' >>4to2<< comment out line
                        'tjz     onTimeCog1, #continueLoop '' >>4to2<< uncomment line
                        or      outa, enableMask1

                        '' >>4to2<< comment out below
checkPwm2               rdlong  onTimeCog2, onTimePtr2 
                        tjz     onTimeCog2, #checkPwm3
                        or      outa, enableMask2
checkPwm3               rdlong  onTimeCog3, onTimePtr3   
                        tjz     onTimeCog3, #continueLoop 
                        or      outa, enableMask3       }
                        '' >>4to2<< comment out above
                                               
                        jmp     #continueLoop                        
'-------------------------------------------------------------------------------

DAT zero                long 0
one                     long 1
negOne                  long -1
'bufferSize              long SPEED_BUFFER_SIZE
encodersInUse           long TOTAL_ENCODERS
destinationIncrement    long %10_0000_0000
destAndSourceIncrement  long %10_0000_0001
sourceIncrement         long 1
timeAddress             long 0
encoderAddress          long 0
'encoderTotal            long 0 
encoderTotal0           long 0-0'[TOTAL_ENCODERS] 
encoderTotal1           long 0
encoderTotal2           long 0 
encoderTotal3           long 0
encoderChange           long 0
encoderQuanta0          long 0-0'[TOTAL_ENCODERS]
encoderQuanta1          long 0
encoderQuanta2          long 0
encoderQuanta3          long 0
previousEncoderTotal0   long 0-0'[TOTAL_ENCODERS]
previousEncoderTotal1   long 0 
previousEncoderTotal2   long 0 
previousEncoderTotal3   long 0
encoderPin0             long 0-0'[TOTAL_ENCODERS]
encoderPin1             long 0
encoderPin2             long 0
encoderPin3             long 0

' first location in each buffer
{buffer0                 long 0
buffer1                 long 0
buffer2                 long 0
buffer3                 long 0 }
pulsePtr                long 0
pulsePtr0               long 0-0'[TOTAL_ENCODERS]
pulsePtr1               long 0
pulsePtr2               long 0
pulsePtr3               long 0

' current location in each buffer
{buffer0Head             long 0
buffer1Head             long 0
buffer2Head             long 0
buffer3Head             long 0   }
'pulseHead               long 0
pulseHead0              long 0-0'[TOTAL_ENCODERS]
pulseHead1              long 0
pulseHead2              long 0
pulseHead3              long 0

tempHead                long 0
bufferTotal0            long 0-0'[TOTAL_ENCODERS]
bufferTotal1            long 0
bufferTotal2            long 0
bufferTotal3            long 0
'pulseTotal              long 0
pulseTotal0             long 0-0'[TOTAL_ENCODERS] 
pulseTotal1             long 0
pulseTotal2             long 0
pulseTotal3             long 0

'pulseBufferSumPtr       long 0
pulseBufferSumPtr0      long 0-0'[TOTAL_ENCODERS]
pulseBufferSumPtr1      long 0
pulseBufferSumPtr2      long 0
pulseBufferSumPtr3      long 0
'lastUpdatedPtr          long 0
lastUpdatedPtr0         long 0-0'[TOTAL_ENCODERS]
lastUpdatedPtr1         long 0
lastUpdatedPtr2         long 0
lastUpdatedPtr3         long 0
headRollover            long 0
enableMask0             long 0-0'[TOTAL_ENCODERS]
enableMask1             long 0
enableMask2             long 0
enableMask3             long 0
onTimePtr0              long 0-0'[TOTAL_ENCODERS]
onTimePtr1              long 0
onTimePtr2              long 0
onTimePtr3              long 0
pwmTimePtr              long 0
pwmTimeCog              long 0
'directionFlagPtr        long 0
directionFlagPtr0       long 0-0'[TOTAL_ENCODERS]
directionFlagPtr1       long 0
directionFlagPtr2       long 0
directionFlagPtr3       long 0
bufferBitsToShift       long 0
'maxBufferIndex          long 0
dataPairAddressPtr      long 0-0

lastTransition0         res TOTAL_ENCODERS
lastEncoderCycle0       res TOTAL_ENCODERS
fullCycleCount0         res TOTAL_ENCODERS
newCycleFlag            res 1
encoderAddressTemp      res 1

channelCountdown        res 1

encoderAddress0         res 1
encoderAddress1         res 1
encoderAddress2         res 1
encoderAddress3         res 1
oldScan                 res 1
oldscan0                res 1
oldscan1                res 1
oldscan2                res 1
oldscan3                res 1
timeCount               res 1
timer                   res 1
speedTimer              res 1
speedTimerTemp          res 1

originalScan            res 1
newscan                 res 1

difference              res 1
oldQuanta               res 1
absoluteTarget          res 1
timePtr                 res 1
'headCountDown           res 1
oldPulse                res 1
temp                    res 1
'lastPulse               res 1  
lastPulse0              res 1
lastPulse1              res 1
lastPulse2              res 1
lastPulse3              res 1
newPulse                res 1
onTimeCog0              res 1
onTimeCog1              res 1
onTimeCog2              res 1
onTimeCog3              res 1
pwmTimer                res 1
pwmTimeTemp             res 1
nowTime                 res 1
tempTotal               res 1
previousTime            res 1
loopTimer               res 1
deltaTime               res 1

              fit
              
DAT {{Terms of Use: MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to miso so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be included in all copies
  or substantial portions of the Software.

  The software is provided "as is", without warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a
  particular purpose and non-infringement. In no even shall the authors or copyright
  holders be liable for any claim, damages or other liability, whether in an action of
  contract, tort or otherwise, arising from, out of or in connection with the software
  or the use of other dealing in the software.
  
}}     