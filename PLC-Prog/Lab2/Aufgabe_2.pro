CoDeSys+7   �                   @        @   2.3.9.41    @/    @                             ��;V +    @                           �,V        �-   @   q   C:\TwinCAT\PLC\LIB\STANDARD.LIB @                                                                                          CONCAT               STR1               ��              STR2               ��                 CONCAT                                         ��66  �   ����           CTD           M             ��           Variable for CD Edge Detection      CD            ��           Count Down on rising edge    LOAD            ��           Load Start Value    PV           ��           Start Value       Q            ��           Counter reached 0    CV           ��           Current Counter Value             ��66  �   ����           CTU           M             ��            Variable for CU Edge Detection       CU            ��       
    Count Up    RESET            ��           Reset Counter to 0    PV           ��           Counter Limit       Q            ��           Counter reached the Limit    CV           ��           Current Counter Value             ��66  �   ����           CTUD           MU             ��            Variable for CU Edge Detection    MD             ��            Variable for CD Edge Detection       CU            ��	       
    Count Up    CD            ��
           Count Down    RESET            ��           Reset Counter to Null    LOAD            ��           Load Start Value    PV           ��           Start Value / Counter Limit       QU            ��           Counter reached Limit    QD            ��           Counter reached Null    CV           ��           Current Counter Value             ��66  �   ����           DELETE               STR               ��              LEN           ��              POS           ��                 DELETE                                         ��66  �   ����           F_TRIG           M             ��
                 CLK            ��           Signal to detect       Q            ��           Edge detected             ��66  �   ����           FIND               STR1               ��              STR2               ��                 FIND                                     ��66  �   ����           INSERT               STR1               ��              STR2               ��              POS           ��                 INSERT                                         ��66  �   ����           LEFT               STR               ��              SIZE           ��                 LEFT                                         ��66  �   ����           LEN               STR               ��                 LEN                                     ��66  �   ����           MID               STR               ��              LEN           ��              POS           ��                 MID                                         ��66  �   ����           R_TRIG           M             ��
                 CLK            ��           Signal to detect       Q            ��           Edge detected             ��66  �   ����           REPLACE               STR1               ��              STR2               ��              L           ��              P           ��                 REPLACE                                         ��66  �   ����           RIGHT               STR               ��              SIZE           ��                 RIGHT                                         ��66  �   ����           RS               SET            ��              RESET1            ��                 Q1            ��
                       ��66  �   ����           SEMA           X             ��                 CLAIM            ��	              RELEASE            ��
                 BUSY            ��                       ��66  �   ����           SR               SET1            ��              RESET            ��                 Q1            ��	                       ��66  �   ����           TOF           M             ��           internal variable 	   StartTime            ��           internal variable       IN            ��       ?    starts timer with falling edge, resets timer with rising edge    PT           ��           time to pass, before Q is set       Q            ��	       2    is FALSE, PT seconds after IN had a falling edge    ET           ��
           elapsed time             ��66  �   ����           TON           M             ��           internal variable 	   StartTime            ��           internal variable       IN            ��       ?    starts timer with rising edge, resets timer with falling edge    PT           ��           time to pass, before Q is set       Q            ��	       0    is TRUE, PT seconds after IN had a rising edge    ET           ��
           elapsed time             ��66  �   ����           TP        	   StartTime            ��           internal variable       IN            ��       !    Trigger for Start of the Signal    PT           ��       '    The length of the High-Signal in 10ms       Q            ��	           The pulse    ET           ��
       &    The current phase of the High-Signal             ��66  �   ����    R    @                                                                                          ADDITION               a                           b                              Addition                                      2SV  @    ����           BITSHIFT           myWord    z]      !               shifted_right            !               shifted_left            !               ormask            !                                �YV  @    ����           COMPUTEMEANVALUE           i            3               sum        	               CurrentStatus   3                      ComputeMeanValue        	               CurrentStatus                            ��V  @   ����           DIVISION               a            &               b            &                  Division                                      �SV  @    ����        
   DOSOMEMATH           a      �?   1                   b       @   2                   sum                            diff                            prod                            quot                                             >WV  @    ����           EDGEDETECTOR           prev             + 
              
   inputValue            +               
   risingEdge            +               fallingEdge            +                        �rV  @    ����           EDGEDETECTORDOUBLE           prev             -               timer            -               
   inputValue            -            	   threshold           -               
   risingEdge            -               fallingEdge            -               double            - 	                       ��V  @   ����        	   FILLARRAY           i            2 	                 data        	               CurrentStatus  2                  out        	               CurrentStatus  2                        E�V  @   ����           FSM           state            (               bEdge                     EdgeDetectorDouble   (               yEdge                     EdgeDetectorDouble   ( 	              count            ( 
           	   led_green             (               led_red             (               period    D       (               st_off           (               st_blink          (               st_red          (               st_green          (                                ?�V  @   ����        	   GENERATOR           t            )               a      �?   1    )               offset           0    )               w      �?   1.0    )               duty      �>   0.25    )               sinus             )               square             ) 	              pwm             ) 
                               ��;V  @   ����           INT_TO_VOLT               input           /                  int_to_volt                                      E�V  @   ����           LOWPASS           out_buf           0    5 
                 data            5               factor            5                  out            5                        ��V  @   ����           MAIN           data0        	               SerialDecode                    data1        	               SerialDecode                    data2        	               SerialDecode                    myarray   	  	                                         i                            sum           0                    prod      �?   1      	              blueEdge                     EdgeDetectorDouble     
           
   yellowEdge                     EdgeDetectorDouble                   myFSM                         FSM                   volt                             myfill              	   fillArray                   filt_iir                  lowpass                   factor    fff?   0.9                    status        	               CurrentStatus                   filt_fir        	               CurrentStatus                                    O�V  @   ����           MULTIPLICATION               a            %               b            %                  Multiplication                                      �SV  @    ����        	   PWMSIGNAL           per            * 
                 a            *               offset            *               w            *               t           *               duty            *               	   PwmSignal                                      J�;V  @�  ����           SERIALDECODE               sw           "                  inLength           "               overrunError            "               framingError            "               parityError            " 	           
   bufferFull            " 
              initAccepted            "               receiveRequest            "               transmitAccepted            "                        ^V  @    ����           SINUSFUNCTION               a            $               offset            $               w            $               t           $                  SinusFunction                                      ��V  @�  ����           SQUARESIGNAL           per            ' 	                 a            '               offset            '               w            '               t           '                  SquareSignal                                      &�V  @�  ����           SUBTRACTION               a            #               b            #                  Subtraction                                      vSV  @    ����           VOLT_TO_INT               input            0                  volt_to_int                                     ��V  @   ����            
 " 	 *   $   '          5   3   (   )   ( �.      K   �.     K   /     K   /     K   0/                 =/         +     ��localhost �ژ=u           (�H �`�ٳ@��� H� 4� l� E1�v�=h�������v���v    s�@             s�@     `� ě� ���   �ճ� ��v�ճF  4� 4� �r� ����    �Φ��           s�@ |�     s�@            s�@     `� ě� `� 4� ě� p`����@� �F�     ,   ,                                                        K         @   �,V1  /*BECKCONFI3*/
        !B @   @   �   �     3                 Standard            	��;V                        VAR_GLOBAL
END_VAR
                                                                                  "   , v  ��             Standard        Main����               ��;V                 $����  1   2                ��������           Standard �,V	�,V      ��������                         	��;V     ����           VAR_CONFIG
END_VAR
                                                                                   '              , �^ �           Global_Variables ��;V	��;V     Lx            �  VAR_GLOBAL
	globalButtonYellow AT%I* : BOOL;
	globalButtonBlue AT%I* : BOOL;
	globalInductiveSensor AT%I* : BOOL;

	intUltrasonicSensor AT%I* : INT;

	globalLightRed AT%Q* : BOOL;
	globalLightGreen AT%Q* : BOOL;

	intAnalogVoltage AT%Q* : INT;

	bufferArray : ARRAY[1.. arrayLength] OF CurrentStatus;

END_VAR

VAR_GLOBAL CONSTANT
	arrayLength : INT := 100;
	F_TASK : INT := 200;
END_VAR                                                                                               '           ,   , �  ��           TwinCAT_Configuration yV	��;V,         H'mm        a  (* Generated automatically by TwinCAT - (read only) *)
VAR_CONFIG
	.globalButtonYellow AT %IX0.0 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 2 (EL1004)^Channel 1^Input} *)
	.globalButtonBlue AT %IX0.1 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 2 (EL1004)^Channel 2^Input} *)
	.globalLightRed AT %QX0.0 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 3 (EL2004)^Channel 1^Output} *)
	.globalLightGreen AT %QX0.1 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 3 (EL2004)^Channel 2^Output} *)
END_VAR                                                                                               '           	   , , , m�           Variable_Configuration �,V	�,V	     Lx��              VAR_CONFIG
END_VAR
                                                                                                 �   |0|0 @|    @Z   MS Sans Serif @       HH':'mm':'ss @      dd'-'MM'-'yyyy   dd'-'MM'-'yyyy HH':'mm':'ss�����                               4     �   ���  �3 ���   � ���     
    @��  ���     @      DEFAULT             System      �   |0|0 @|    @Z   MS Sans Serif @       HH':'mm':'ss @      dd'-'MM'-'yyyy   dd'-'MM'-'yyyy HH':'mm':'ss�����                      )   HH':'mm':'ss @                             dd'-'MM'-'yyyy @       '   6   ,     A�           CurrentStatus p�V	p�V      p;�w�A8C          TYPE CurrentStatus :
STRUCT
	buttonYellow : BOOL;
	buttonBlue : BOOL;
	lightGreen : BOOL;
	lightRed : BOOL;
	inductiveSensor : BOOL;
	intUltrasonicSensor : INT;
	voltageUltrasonicSensor : REAL;
	intAnalogOutput : INT;
	voltageAnalogOutput : REAL;
END_STRUCT
END_TYPE                 , ' 	�            Addition 2SV	2SV      ��            R   FUNCTION Addition : REAL
VAR_INPUT
	a : REAL;
	b : REAL;
END_VAR
VAR
END_VAR   Addition := a + b;               !   , � � B�           BitShift ZV	�YV                      {   PROGRAM BitShift
VAR
	myWord : WORD := 16#5D7A;
	shifted_right : WORD;
	shifted_left : DWORD;
	ormask : WORD;
END_VARm   shifted_right := SHR(myword, 3);
shifted_left := SHL(WORD_TO_DWORD(myword),3);
ormask := myword OR 16#F000;               3   , � � �           ComputeMeanValue �V	��V      teorub;        n   FUNCTION ComputeMeanValue : CurrentStatus
VAR_INPUT
END_VAR
VAR
	i : INT ;
	sum : CurrentStatus;
END_VAR�  FOR i := 1 TO arrayLength BY 1 DO
	sum.intAnalogOutput := sum.intAnalogOutput+ bufferArray[i].intAnalogOutput;
	sum.intUltrasonicSensor := sum.intUltrasonicSensor+ bufferArray[i].intUltrasonicSensor;
	sum.voltageAnalogOutput := sum.voltageAnalogOutput+ bufferArray[i].voltageAnalogOutput;
	sum.voltageUltrasonicSensor := sum.voltageUltrasonicSensor+ bufferArray[i].voltageUltrasonicSensor;
END_FOR
ComputeMeanValue.intAnalogOutput := sum.intAnalogOutput / arrayLength;
ComputeMeanValue.intUltrasonicSensor := sum.intUltrasonicSensor / arrayLength;
ComputeMeanValue.voltageAnalogOutput := sum.voltageAnalogOutput / arrayLength;
ComputeMeanValue.voltageUltrasonicSensor := sum.voltageUltrasonicSensor / arrayLength;               &   , 0� �           Division TV	�SV      J ���        R   FUNCTION Division : REAL
VAR_INPUT
	a : REAL;
	b : REAL;
END_VAR
VAR
END_VARA   IF b = 0 THEN
	Division := 0;
ELSE
	Division := a / b;
END_IF                  , *4 ��        
   DoSomeMath >WV	>WV                      }   PROGRAM DoSomeMath
VAR
	a : REAL := 1;
	b : REAL := 2;
	sum : REAL;
	diff : REAL;
	prod : REAL;
	quot : REAL;
END_VAR�   sum := Addition(a:=a, b:=b);
diff := Subtraction(a:=a, b:=b);
prod := Multiplication(a:=a, b:=b);
quot := Division(a:=a, b:=b);               +   , a ��           EdgeDetector zV	�rV                   �   FUNCTION_BLOCK EdgeDetector
VAR_INPUT
	inputValue : BOOL;
END_VAR
VAR_OUTPUT
	risingEdge : BOOL;
	fallingEdge : BOOL;
END_VAR
VAR
	prev : BOOL := FALSE;
END_VAR�   IF inputValue > prev THEN
	risingEdge := TRUE;
	fallingEdge := FALSE;
ELSIF inputValue < prev THEN
	risingEdge := FALSE;
	fallingEdge := TRUE;
ELSE
	risingEdge := FALSE;
	fallingEdge := FALSE;
END_IF
prev := inputValue;               -   ,    J�           EdgeDetectorDouble ��V	��V                      �   FUNCTION_BLOCK EdgeDetectorDouble
VAR_INPUT
	inputValue : BOOL;
	threshold : INT;
END_VAR
VAR_OUTPUT
	risingEdge : BOOL;
	fallingEdge : BOOL;
	double : BOOL;
END_VAR
VAR
	prev : BOOL := FALSE;
	timer : INT := 0;
END_VAR�  IF inputValue > prev THEN
	risingEdge := TRUE;
	fallingEdge := FALSE;
	IF timer < threshold THEN
		double := TRUE;
	ELSE
		double := FALSE;
	END_IF
	timer := 0;
ELSIF inputValue < prev THEN
	risingEdge := FALSE;
	fallingEdge := TRUE;
	IF timer < 32767 THEN
		timer := timer + 1;
	END_IF
	double := FALSE;
ELSE
	risingEdge := FALSE;
	fallingEdge := FALSE;
	IF timer < 32767 THEN
		timer := timer + 1;
	END_IF
	double := FALSE;
END_IF
prev := inputValue;               2   ,   W�        	   fillArray E�V	E�V                      �   FUNCTION_BLOCK fillArray
VAR_INPUT
	data : CurrentStatus;
END_VAR
VAR_OUTPUT
	out: CurrentStatus;
END_VAR
VAR
	i : INT;
END_VARq   IF i < arrayLength THEN
	i := i + 1;
ELSE
	i := 1;
END_IF

out := bufferArray[i] ;
bufferArray[i] := data;               (   ,   =           FSM ?�V	?�V       ���        j  FUNCTION_BLOCK FSM
VAR_INPUT
END_VAR
VAR_OUTPUT
END_VAR
VAR
	state : INT := 0;
	bEdge : EdgeDetectorDouble;
	yEdge : EdgeDetectorDouble;
	count : INT := 0;
	led_green : BOOL := FALSE;
	led_red : BOOL := FALSE;
	period : INT := 68;
END_VAR
VAR CONSTANT
	st_off : INT := 0;
	st_blink : INT := 1;
	st_red : INT :=2 ;
	st_green : INT := 3;
END_VAR�  bEdge(inputValue := globalButtonBlue, threshold := 40);
yEdge(inputValue := globalButtonYellow, threshold := 40);
CASE state OF
	st_off:
		led_red := FALSE;
		led_green := FALSE;
		IF bEdge.risingEdge = TRUE THEN
			state := st_blink;
			IF bEdge.double = TRUE THEN
				period := 34;
			ELSE
				period := 68;
			END_IF
		ELSIF yEdge.risingEdge = TRUE THEN
			state := st_red;
		END_IF

	st_blink:
		IF count >= period THEN
			count := 0;
			led_red := NOT led_red;
			led_green := NOT led_green;
		ELSE
			count := count + 1;
		END_IF
		IF bEdge.risingEdge = TRUE THEN
			state := st_off;
		ELSIF yEdge.risingEdge = TRUE THEN
			state := st_red;
		END_IF
	st_red:
		led_red := TRUE;
		led_green := FALSE;
		IF bEdge.risingEdge = TRUE THEN
			state := st_blink;
			led_red := FALSE;
			led_green := FALSE;
		ELSIF yEdge.risingEdge = TRUE THEN
			state := st_green;
		END_IF
	st_green:
		led_red := FALSE;
		led_green := TRUE;
		IF bEdge.risingEdge = TRUE THEN
			state := st_blink;
			led_red := FALSE;
			led_green := FALSE;
		ELSIF yEdge.risingEdge = TRUE THEN
			state := st_red;
		END_IF
END_CASE

		globalLightRed :=led_red ;
		globalLightGreen := led_green;               )   ,  	 4�        	   Generator ��;V	��;V                      �   PROGRAM Generator
VAR
	t : UINT := 0;
	a : REAL := 1;
	offset : REAL := 0;
	w : REAL := 1.0;
	duty : REAL := 0.25;
	sinus : REAL;
	square : REAL;
	pwm : REAL;
END_VAR�   IF t <= REAL_TO_INT(w*F_TASK) THEN
	t := t + 1;
ELSE
	t := 0;
END_IF
sinus := SinusFunction(a, offset, w, t);
square := SquareSignal(a, offset, w, t);
pwm := PwmSignal(a, offset, w, t, duty);
               /   , B B ��           int_to_volt �V	E�V       ���        L   FUNCTION int_to_volt : REAL
VAR_INPUT
	input : INT;
END_VAR
VAR
END_VAR6   int_to_volt := 10 - INT_TO_REAL(input) * 10 / 16#7FFF;               5   , � * ��           lowpass ΩV	��V                      �   FUNCTION_BLOCK lowpass
VAR_INPUT
	data : REAL;
	factor : REAL;
END_VAR
VAR_OUTPUT
	out : REAL;
END_VAR
VAR
	out_buf : REAL := 0;
END_VARC   out_buf := factor * out_buf + (1 - factor) * data;
out := out_buf;                   ,   x/           MAIN O�V	O�V                      �  PROGRAM MAIN
VAR
	data0 : SerialDecode;
	data1 : SerialDecode;
	data2 : SerialDecode;
	myarray : ARRAY[0..9] OF REAL;
	i : INT;
	sum : REAL := 0;
	prod : REAL := 1;
	blueEdge : EdgeDetectorDouble;
	yellowEdge : EdgeDetectorDouble;
	myFSM : FSM;
	volt : REAL;
	myfill : fillArray;
	filt_iir : lowpass;
	factor : REAL := 0.9;
	status : CurrentStatus;
	filt_fir : CurrentStatus;
END_VAR  DoSomeMath();

BitShift();

data0(sw := 16#1D06);
data1(sw := 16#534C);
data2(sw := 16#C705);
FOR i := 0 TO 9 BY 1 DO
myarray[i] := i+1;
END_FOR
sum := 0;
prod := 1;
FOR i := 0 TO 9 BY 1 DO
	sum := Addition(sum, myarray[i]);
END_FOR
i := 0;
WHILE i <= 9 DO
	prod := Multiplication(prod, myarray[i]);
	i := i + 1;
END_WHILE

Generator();

blueEdge(inputValue := globalButtonBlue, threshold := 40);
yellowEdge(inputValue := globalButtonYellow, threshold := 40);
globalLightRed:= blueEdge.risingEdge OR yellowEdge.fallingEdge OR yellowEdge.risingEdge OR blueEdge.fallingEdge;
globalLightGreen := yellowEdge.double OR blueEdge.double;

status.buttonBlue := globalButtonBlue;
status.buttonYellow := globalButtonYellow;
status.inductiveSensor := globalInductiveSensor;
status.intUltrasonicSensor := intUltrasonicSensor;
status.voltageUltrasonicSensor := int_to_volt(intUltrasonicSensor);

myFSM();
volt := int_to_volt(intUltrasonicSensor);
myfill(data := status);
filt_iir(data := volt, factor := factor);
filt_fir := ComputeMeanValue();


volt := filt_iir.out;

intAnalogVoltage := volt_to_int(volt);

status.lightGreen := globalLightGreen;
status.lightRed := globalLightRed;
status.voltageAnalogOutput := volt;
status.intAnalogOutput :=intAnalogVoltage;               %   ,    �            Multiplication �SV	�SV      Te\*ic          X   FUNCTION Multiplication : REAL
VAR_INPUT
	a : REAL;
	b : REAL;
END_VAR
VAR
END_VAR   Multiplication := a * b;               *   ,  �O        	   PwmSignal g�;V	J�;V                      �   FUNCTION PwmSignal : REAL
VAR_INPUT
	a : REAL;
	offset : REAL;
	w : REAL;
	t : INT;
	duty : REAL;
END_VAR
VAR
	per : INT;
END_VAR�   per := REAL_TO_INT(F_TASK / w);
IF duty*per >= per THEN
	PwmSignal := offset + a;
ELSIF duty*per <=0 THEN
	PwmSignal := offset - a;
ELSIF (t MOD per) < duty*per THEN
	PwmSignal := offset + a;
ELSE
	PwmSignal := offset - a;
END_IF               "   , �  X           SerialDecode x}V	^V                        FUNCTION_BLOCK SerialDecode
VAR_INPUT
	sw : WORD;
END_VAR
VAR_OUTPUT
	inLength : INT;
	overrunError : BOOL;
	framingError : BOOL;
	parityError : BOOL;
	bufferFull : BOOL;
	initAccepted : BOOL;
	receiveRequest : BOOL;
	transmitAccepted : BOOL;
END_VAR
VAR
END_VAR�   inLength := WORD_TO_INT(SHR((sw AND 16#FF00), 8));
overrunError := sw.6;
framingError := sw.5;
parityError := sw.4;
bufferFull := sw.3;
initAccepted := sw.2;
receiveRequest := sw.1;
transmitAccepted := sw.0;

               $   , � X iq           SinusFunction ��V	��V       1Osu:=        s   FUNCTION SinusFunction : REAL
VAR_INPUT
	a : REAL;
	offset : REAL;
	w : REAL;
	t : INT;
END_VAR
VAR
END_VARH   SinusFunction := a * SIN(INT_TO_REAL(t)*w /F_TASK *2.0*3.1415) + offset;               '   , I �            SquareSignal ��V	&�V      �                 FUNCTION SquareSignal : REAL
VAR_INPUT
	a : REAL;
	offset : REAL;
	w : REAL;
	t : INT;
END_VAR
VAR
	per : INT;
END_VAR�   per := REAL_TO_INT(F_TASK / w);
IF (t MOD per) < (per / 2) THEN
	SquareSignal := offset + a;
ELSE
	SquareSignal := offset - a;
END_IF               #   , � �           Subtraction �SV	vSV                      U   FUNCTION Subtraction : REAL
VAR_INPUT
	a : REAL;
	b : REAL;
END_VAR
VAR
END_VAR   Subtraction := a - b;               0   , RJ ��           volt_to_int 
�V	��V                      L   FUNCTION volt_to_int : INT
VAR_INPUT
	input : REAL;
END_VAR
VAR
END_VAR1   volt_to_int := REAL_TO_INT(input * 16#7FFF / 10);                 ����        ��         "   STANDARD.LIB 5.6.98 11:03:02 @V�w5      CONCAT @                	   CTD @        	   CTU @        
   CTUD @           DELETE @           F_TRIG @        
   FIND @           INSERT @        
   LEFT @        	   LEN @        	   MID @           R_TRIG @           REPLACE @           RIGHT @           RS @        
   SEMA @           SR @        	   TOF @        	   TON @           TP @              Global Variables 0 @                           FSE
E_I           2                ����������������  
             ����  EtrC)^rm        ����  (iMaal))                      POUs                 Addition                     BitShift  !                   ComputeMeanValue  3                   Division  &                
   DoSomeMath                     EdgeDetector  +                   EdgeDetectorDouble  -                	   fillArray  2                   FSM  (               	   Generator  )                   int_to_volt  /                   lowpass  5                   MAIN                      Multiplication  %                	   PwmSignal  *                   SerialDecode  "                   SinusFunction  $                   SquareSignal  '                   Subtraction  #                   volt_to_int  0   ����          
   Data types                 CurrentStatus  6   ����             Visualizations  ����              Global Variables                Global_Variables                     TwinCAT_Configuration  ,                   Variable_Configuration  	   ����                                                              �,V                         	   localhost            P      	   localhost            P      	   localhost            P             �}zs