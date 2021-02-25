%INTRODUCTION 
fprintf(['\nThis program will perform all of the calculations required for determining spur gear and shaft specifications.\n']);
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause;


%STEP 1 - PERFORMANCE SPECIFICATIONS AND CONSTRAINTS 
fprintf('\nSTEP 1: Performance Specifications\n');
%Motor Performance Specifications 
NLspeed = 4688; %rpm
StallTorque = 31.38128;%(TorqueConst/1000) * NomOpVoltage / ArmatureResistance;Nm
Pin = 114.59; %12W *60/2/pi() 
%Design Constraints and Assumptions 
OutputTorque = 2; % N*m
OutputSpeed = 60; % rpm
DutyReq = (8 * 5 * 50); % hours/year
GearEfficiency = 0.98; % two stage has efficiency of 97-99%
Pitch = 2;

%Output variables in a table 
fprintf('\nMotor Performance Specifications');
Table1 = table(categorical({'No-Load Speed';'Body Length';}),...
           [NLspeed;StallTorque],...
          categorical({'rpm';'Nm';}),...
          'VariableNames',{'Specification','Value','Units'})
fprintf('Design Constraints and Assumptions');
Table2 = table(categorical({'Operating Torque';'Output Speed';'Duty Requirements';'Gear Efficiency';'Number of Reduction Stages';}),...
           [OutputTorque;OutputSpeed;DutyReq;GearEfficiency;2],...
          categorical({'Nm';'rpm';'hours/year';' - ';' - ';}),...
          'VariableNames',{'Specification','Value','Units'})
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause;


%STEP 2 - TORQUE SPEED PLOT
fprintf('\nSTEP 2: Torque - Speed Plot\n');
fprintf(['\nNo-Load Speed = Voltage Constant * Nominal Operating Voltage\nNo-Load Speed = %i',...
        ' rpm\n'],NLspeed)
%Stall Torque 
fprintf(['\nStall Torque = Nominal Operating Voltage  * Armature Resistance\nStall Torque = %i',...
        ' Nm\n'],StallTorque)
%Equation of the line 
m = -StallTorque / NLspeed; %Slope 
x = 0:NLspeed; %X range, must be large enough to include the no load speed  
y = m*x + StallTorque; %Equation of the line, Tin = m*W_in + T_stall 
fprintf(['\nEquation of the torque-speed plot:\ny = mx + b\nm = -Stall Torque / No-Load Speed = %i',...
        '\nb = Stall Torque = %i','\nTorque = m * Speed + b\nTorque = %i',' * speed + %i', '\n\n'],m,StallTorque,m,StallTorque)
plot(y);
title('Torque - Speed Plot');
xlabel('Speed [rpm]') ;
ylabel('Torque [Nm]');
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause; 
close all;


%STEP 3 - INPUT SPEEDS THAT RESULTS IN REQUIRED POWER
fprintf('\nSTEP 3: Input Speeds that Provide Required Power\n');
%Output Power Required
Pout = Pin*GearEfficiency%(OutputSpeed) * OutputTorque; %Nm*rpm
fprintf(['\nOutput Power = Output Speed * Output Torque = %i',...
        'watts\n'],Pout)
% Pout = Pin * Efficiency \nTorque = %i',' * speed + %i', '\n\n'],m,StallTorque,
fprintf(['\nOutput Power = Input Power * Gear Efficiency\nOutput Torque * Output Speed = Input Torque * Input Speed * Gear Efficiency\nSubstitute Equation from step 2: Input Torque = %i',...
        ' * speed + %i', '\n\nThe resulting qudratic was solved, representing the two possible input speeds that result in an output torque of 55 Nm:\n'],m,StallTorque)
A = (GearEfficiency^2) * m;
B = (GearEfficiency^2) * StallTorque;
C = -Pout;
InputSpeedOne = (-B + sqrt( (B^2) - 4*A*C)) / (2*A);
InputSpeedTwo = (-B - sqrt( (B^2) - 4*A*C)) / (2*A);
fprintf(['Input Speed One: %i',...
        ' rad/s\nInput Speed Two: %i', ' rad/s.\nInput speed one will be used as it will lead to a smaller gear ratio.\n\n'],InputSpeedOne,InputSpeedTwo)
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause;


%STEP 4 - GEAR SPECIFICATIONS
fprintf('\nSTEP 4: Gear Specifcations\n');
TotalReduction = 1/round(1/(InputSpeedOne / (OutputSpeed)),0); %rounded
fprintf('\nThe total reduction from input speed to output speed is by a factor of: %i',TotalReduction);
%Calculation of theoretical gear ratio
TheoreticalGearRatio = sqrt(TotalReduction);
GearRatio1 = 1/(floor(sqrt(1/TotalReduction)));
GearRatio2 = (TotalReduction/GearRatio1);

fprintf('\nAn Evenly split theoretical Gear Ratio would be: %i',TheoreticalGearRatio);
fprintf('\nFor better teeth selection, Stage 1 should have a gear ratio of: %i',GearRatio1);
fprintf('\nFor better teeth selection, Stage 2 should have a gear ratio of: %i',GearRatio2);
%From gear catalogue 
PressureAngle = 20;%degrees, from textbook 
k = 1; %full-depth gear teeth
%Minimum number of teeth for pinion, eq 13-11
A = (sind(PressureAngle).^2);
B = (1 + (2*TheoreticalGearRatio));
C = (TheoreticalGearRatio + sqrt(TheoreticalGearRatio.^2 + (B * A)));
MinPinionTeeth = (((2*k) / (B*A)) * C);
fprintf(['\nThe minimum number of teeth for the pinion was calculated using the interference equations.\nIt was determined that the pinion must have at least %i',' teeth to avoid interference.\n'],MinPinionTeeth);

%Selected Gears from gear catalogue 
fprintf('\nSTAGE 1: ');
prompt = 'Enter the number of teeth for the pinion (Stage 1): ';
PinionTeeth1 = input(prompt);
GearTeeth1 = PinionTeeth1/GearRatio1; 
fprintf(['\nThe gear(Stage 1)should have %i teeth\n'],GearTeeth1);
PinionDiameter1 = PinionTeeth1*Pitch/pi();
GearDiameter1 = GearTeeth1*Pitch/pi();
fprintf(['The pinion(Stage 1)should have %i mm pitch diameter\n'],PinionDiameter1);
fprintf(['The gear(Stage 1)should have %i mm pitch diameter\n'],GearDiameter1);


%Selected Gears from gear catalogue 
fprintf('\nSTAGE 2: ');
prompt = 'Enter the number of teeth for the pinion (Stage 2): ';
PinionTeeth2 = input(prompt);
GearTeeth2 = PinionTeeth2/GearRatio2; 
fprintf(['\nThe gear(Stage 2)should have %i teeth\n'],GearTeeth2);
PinionDiameter2 = PinionTeeth2*Pitch/pi();
GearDiameter2 = GearTeeth2*Pitch/pi();
fprintf(['The pinion(Stage 2)should have %i mm pitch diameter\n'],PinionDiameter2);
fprintf(['The gear(Stage 2)should have %i mm pitch diameter\n'],GearDiameter2);


%Output a table with gear specifications 
fprintf('\nSTAGE 1 - Gear Specifications: ');
Table3 = table(categorical({'Gear Ratio';'Pinion Teeth';'Gear Teeth';'Pinion Pitch Diameter';'Gear Pitch Diameter';}),...
           [GearRatio1;PinionTeeth1;GearTeeth1;PinionDiameter1;GearDiameter1;],...
          categorical({' - ';' - ';' - ';'mm';'mm';}),...
          'VariableNames',{'Specification','Value','Units'})
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause 

%Output a table with gear specifications 
fprintf('\nSTAGE 2 - Gear Specifications: ');
Table3 = table(categorical({'Gear Ratio';'Pinion Teeth';'Gear Teeth';'Pinion Pitch Diameter';'Gear Pitch Diameter';}),...
           [GearRatio2;PinionTeeth2;GearTeeth2;PinionDiameter2;GearDiameter2;],...
          categorical({' - ';' - ';' - ';'mm';'mm';}),...
          'VariableNames',{'Specification','Value','Units'})
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause 

%A few more calculations
InputTorque = m*InputSpeedOne + StallTorque;
ActualOutputSpeed = (InputSpeedOne * (1/GearRatio1) * (1/GearRatio2));
ActualOutputTorque = ((GearEfficiency.^2) * InputTorque * (GearRatio1) * (GearRatio2));
MidSpeed = InputSpeedOne / (GearRatio1);
%Output a table with gear specifications 
fprintf('\nSpeed and Torque Results: ');
Table3 = table(categorical({'Input Speed';'Input Torque';'Actual Output Speed';'Actual Output Torque';'Middle Shaft Speed';}),...
           [InputSpeedOne;InputTorque;ActualOutputSpeed;ActualOutputTorque;MidSpeed;],...
          categorical({'rpm';'Nm';'rpm';'Nm';'rpm';}),...
          'VariableNames',{'Specification','Value','Units'})
fprintf(['----------------------------------------------------------------------------------------------------\n']);
pause 



%