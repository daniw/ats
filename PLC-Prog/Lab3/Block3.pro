CoDeSys+'                      @        @   2.3.9.41    @/    @                             У V +    @      SE	rindg             еV        t   @   q   C:\TwinCAT\PLC\LIB\STANDARD.LIB @                                                                                          CONCAT               STR1               §џ              STR2               §џ                 CONCAT                                         ды66     џџџџ           CTD           M             §џ           Variable for CD Edge Detection      CD            §џ           Count Down on rising edge    LOAD            §џ           Load Start Value    PV           §џ           Start Value       Q            §џ           Counter reached 0    CV           §џ           Current Counter Value             ды66     џџџџ           CTU           M             §џ            Variable for CU Edge Detection       CU            §џ       
    Count Up    RESET            §џ           Reset Counter to 0    PV           §џ           Counter Limit       Q            §џ           Counter reached the Limit    CV           §џ           Current Counter Value             ды66     џџџџ           CTUD           MU             §џ            Variable for CU Edge Detection    MD             §џ            Variable for CD Edge Detection       CU            §џ	       
    Count Up    CD            §џ
           Count Down    RESET            §џ           Reset Counter to Null    LOAD            §џ           Load Start Value    PV           §џ           Start Value / Counter Limit       QU            §џ           Counter reached Limit    QD            §џ           Counter reached Null    CV           §џ           Current Counter Value             ды66     џџџџ           DELETE               STR               §џ              LEN           §џ              POS           §џ                 DELETE                                         ды66     џџџџ           F_TRIG           M             §џ
                 CLK            §џ           Signal to detect       Q            §џ           Edge detected             ды66     џџџџ           FIND               STR1               §џ              STR2               §џ                 FIND                                     ды66     џџџџ           INSERT               STR1               §џ              STR2               §џ              POS           §џ                 INSERT                                         ды66     џџџџ           LEFT               STR               §џ              SIZE           §џ                 LEFT                                         ды66     џџџџ           LEN               STR               §џ                 LEN                                     ды66     џџџџ           MID               STR               §џ              LEN           §џ              POS           §џ                 MID                                         ды66     џџџџ           R_TRIG           M             §џ
                 CLK            §џ           Signal to detect       Q            §џ           Edge detected             ды66     џџџџ           REPLACE               STR1               §џ              STR2               §џ              L           §џ              P           §џ                 REPLACE                                         ды66     џџџџ           RIGHT               STR               §џ              SIZE           §џ                 RIGHT                                         ды66     џџџџ           RS               SET            §џ              RESET1            §џ                 Q1            §џ
                       ды66     џџџџ           SEMA           X             §џ                 CLAIM            §џ	              RELEASE            §џ
                 BUSY            §џ                       ды66     џџџџ           SR               SET1            §џ              RESET            §џ                 Q1            §џ	                       ды66     џџџџ           TOF           M             §џ           internal variable 	   StartTime            §џ           internal variable       IN            §џ       ?    starts timer with falling edge, resets timer with rising edge    PT           §џ           time to pass, before Q is set       Q            §џ	       2    is FALSE, PT seconds after IN had a falling edge    ET           §џ
           elapsed time             ды66     џџџџ           TON           M             §џ           internal variable 	   StartTime            §џ           internal variable       IN            §џ       ?    starts timer with rising edge, resets timer with falling edge    PT           §џ           time to pass, before Q is set       Q            §џ	       0    is TRUE, PT seconds after IN had a rising edge    ET           §џ
           elapsed time             ды66     џџџџ           TP        	   StartTime            §џ           internal variable       IN            §џ       !    Trigger for Start of the Signal    PT           §џ       '    The length of the High-Signal in 10ms       Q            §џ	           The pulse    ET           §џ
       &    The current phase of the High-Signal             ды66     џџџџ    R    @                                                                                          EDGEDETECTOR           prev             $ 
              
   inputValue            $               
   risingEdge            $               fallingEdge            $                        ЅиV  @    џџџџ           EDGEDETECTORDOUBLE           prev             %               timer            %               
   inputValue            %            	   threshold           %               
   risingEdge            %               fallingEdge            %               double            % 	                       ЈиV  @    џџџџ           MAIN     	      state            	   ST_Tresor                    b_y                     EdgeDetectorDouble                    b_b                     EdgeDetectorDouble                    code   	                                          index                           t_delay    ш                   
   flag_wrong               	              errorCnt              
              duration    ш  @                                   %ўV  @   џџџџ            
 "      #       ( _      K   m     K   {     K        K                    Ћ         +     КЛlocalhost ЯкЕv          шwH Я`ЯpX8Яд 8д $ж \е E1,wРЪ2 ўџџџу'wтр'w    sё@             sё@     Pз Фэ g   hе р'wpF  $ж $ж Еrћ џџџџ     4xе           sё@ lе     sё@            sё@     Pз Фэ Pз $ж Фэ p`џџџџ0ж ЭFэ     ,   ,                                                        K         @   еV  /*BECKCONFI3*/
        !B @   @           3                  Standard            	У V     З               VAR_GLOBAL
END_VAR
                                                                                  "   ,   л'             Standard         MAINџџџџ               У V                 $ћџџџ                       аџ ТYш           Standard еV	еV       К  #ъ                         	У V                      VAR_CONFIG
END_VAR
                                                                                   '              , c=9           Global_Variables У V	У V     Lx5           ц   VAR_GLOBAL
	btn_yellow AT %I* : BOOL;
	btn_blue AT %I* : BOOL;
	led_green AT %Q*: BOOL;
	led_red AT %Q*: BOOL;
	intVoltage AT %Q* : INT;
END_VAR
VAR_GLOBAL CONSTANT
	CODESIZE : INT := 4;
	ERRORCNTMAX : INT := 20;
END_VAR                                                                                               '           &   , K K о           TwinCAT_Configuration  мV	У V&         H'mm        в  (* Generated automatically by TwinCAT - (read only) *)
VAR_CONFIG
	.btn_yellow AT %IX4.0 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 2 (EL1004)^Channel 1^Input} *)
	.btn_blue AT %IX0.1 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 2 (EL1004)^Channel 2^Input} *)
	.led_green AT %QX0.0 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 3 (EL2004)^Channel 2^Output} *)
	.led_red AT %QX0.1 : BOOL;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 3 (EL2004)^Channel 1^Output} *)
	.intVoltage AT %QB4 : INT;	(*  ~ {LinkedWith:TIID^Device 1 (EtherCAT)^Term 1 (EK1200)^Term 5 (EL4034)^AO Outputs Channel 1^Analog output} *)
END_VAR                                                                                               '           	     Bod>/Lun           Variable_Configuration еV	еV	     Lxџџ              VAR_CONFIG
END_VAR
                                                                                                    |0|0 @|    @Z   MS Sans Serif @       HH':'mm':'ss @      dd'-'MM'-'yyyy   dd'-'MM'-'yyyy HH':'mm':'ssѓџџџ                               4     џ   џџџ  Ь3 џџџ   џ џџџ     
    @џ  џџџ     @      DEFAULT             System         |0|0 @|    @Z   MS Sans Serif @       HH':'mm':'ss @      dd'-'MM'-'yyyy   dd'-'MM'-'yyyy HH':'mm':'ssѓџџџ                      )   HH':'mm':'ss @                             dd'-'MM'-'yyyy @       '   #   , o [        	   ST_Tresor iъV	JжV      : OL= LS           TYPE ST_Tresor :
	(
		ST_init,
		ST_close,
		ST_wrong,
		ST_check,
		ST_open,
		ST_progInit,
		ST_prog,
		ST_progExit
);
END_TYPE              $   ,   Х           EdgeDetector ЈиV	ЅиV      ib*.b@cr        Њ   FUNCTION_BLOCK EdgeDetector
VAR_INPUT
	inputValue : BOOL;
END_VAR
VAR_OUTPUT
	risingEdge : BOOL;
	fallingEdge : BOOL;
END_VAR
VAR
	prev : BOOL := FALSE;
END_VARх   IF inputValue > prev THEN
	risingEdge := TRUE;
	fallingEdge := FALSE;
ELSIF inputValue < prev THEN
	risingEdge := FALSE;
	fallingEdge := TRUE;
ELSE
	risingEdge := FALSE;
	fallingEdge := FALSE;
END_IF
prev := inputValue;               %   , {E Мв           EdgeDetectorDouble ЈиV	ЈиV                      ш   FUNCTION_BLOCK EdgeDetectorDouble
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
END_VARм  IF inputValue > prev THEN
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
prev := inputValue;                   , ќџћџGZ           MAIN %ўV	%ўV      etti*)EN        %  PROGRAM MAIN
VAR
	state : ST_Tresor;
	b_y : EdgeDetectorDouble;
	b_b : EdgeDetectorDouble;
	code : ARRAY[1..CODESIZE] OF BOOL;
	index : INT := 1;
	t_delay : INT := duration;
	flag_wrong : BOOL := FALSE;
	errorCnt : INT := 0;
END_VAR
VAR CONSTANT
	duration : INT := 1000;
END_VAR
?  b_y(inputValue := btn_yellow, threshold  := 30);
b_b(inputValue := btn_blue, threshold  := 30);
CASE state OF
	ST_init:
		state := ST_close;
		code[1] := TRUE;
		code[2] := TRUE;
		code[3] := FALSE;
		code[4] := TRUE;
		errorCnt := 0;
	ST_close:
		led_green := FALSE;
		led_red := FALSE;
		index := 1;
		state := ST_check;
		flag_wrong := FALSE;
	ST_wrong:
		IF t_delay > 0THEN
			T_delay := T_delay - 1;
			led_red := TRUE;
		ELSE
			T_delay := duration;
			led_red := FALSE;
			state := ST_close;
		END_IF
	ST_check:
		IF b_b.risingEdge THEN
			IF code[index] = FALSE THEN
				flag_wrong := TRUE;
			END_IF
			index := index +1;
		ELSIF b_y.risingEdge THEN
			IF code[index] = TRUE THEN
				flag_wrong := TRUE;
			END_IF
			index := index +1;
		END_IF
		IF index > CODESIZE THEN
			IF flag_wrong = TRUE THEN
				state := ST_wrong;
				t_delay := duration;
				IF errorCnt < ERRORCNTMAX THEN
					errorCnt := errorCnt + 1;
				END_IF
			ELSE
				state := ST_open;
				t_delay := duration;
				errorCnt := 0;
			END_IF
		END_IF
		IF b_b.double OR b_y.double THEN
			state := ST_close;
		END_IF
	ST_open:
		IF t_delay > 0THEN
			T_delay := T_delay - 1;
			led_green := TRUE;
		ELSE
			T_delay := duration;
			led_green := FALSE;
			state := ST_close;
		END_IF
		IF btn_yellow AND btn_blue THEN
			state := ST_progInit;
			t_delay := duration / 5;
		END_IF
	ST_progInit:
		t_delay := t_delay -1;
		led_green := TRUE;
		led_red := TRUE;
		IF t_delay = 0 THEN
			state := ST_prog;
			led_green := FALSE;
			led_red := FALSE;
			index := 1;
		END_IF
	ST_prog:
		IF b_b.risingEdge THEN
			code[index] := TRUE;
			index := index + 1;
		ELSIF b_y.risingEdge THEN
			code[index] := FALSE;
			index := index + 1;
		END_IF
		IF index > CODESIZE THEN
			state := ST_progExit;
			t_delay := duration / 5;
		END_IF
	ST_progExit:
		t_delay := t_delay -1;
		led_green := TRUE;
		led_red := TRUE;
		IF t_delay = 0 THEN
			state := ST_close;
		END_IF
END_CASE
intVoltage := REAL_TO_INT(INT_TO_REAL(errorCnt) * 16#7FFF / ERRORCNTMAX);                 §џџџ  % [ I         "   STANDARD.LIB 5.6.98 12:03:02 @VТw5      CONCAT @                	   CTD @        	   CTU @        
   CTUD @           DELETE @           F_TRIG @        
   FIND @           INSERT @        
   LEFT @        	   LEN @        	   MID @           R_TRIG @           REPLACE @           RIGHT @           RS @        
   SEMA @           SR @        	   TOF @        	   TON @           TP @              Global Variables 0 @                          - ,               2                џџџџџџџџџџџџџџџџ  
             њџџџ                 јџџџ  p 8`>pЂ                      POUs                EdgeDetector  $                   EdgeDetectorDouble  %                   MAIN      џџџџ           
   Data types             	   ST_Tresor  #   џџџџ             Visualizations  џџџџ              Global Variables                Global_Variables                     TwinCAT_Configuration  &                   Variable_Configuration  	   џџџџ                                                              еV                         	   localhost            P      	   localhost            P      	   localhost            P             И&