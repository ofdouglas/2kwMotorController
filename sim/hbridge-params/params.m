

function params()
% Calculation of H-bridge parameters, for the ON-BRAKE mode
% 

  
% Battery voltage
Vbus = 24:6:48;

% Current and duty cycle vectors  
I = zeros(1,length(Vbus));
D = zeros(1,length(Vbus));

% Ampflow E30-400 motor parameters
L = 120e-6;             % Motor inductance
Rm = 0.1;		% Armature resistance
Vg = 15;		% Generator voltage (arbitrary)

% H-bridge parameters (using IRFP4468 MOSFET) 
f = 20e3;               % PWM frequency
tr = 100e-9;		% Rise time
N = 1;                  % N MOSFETs in parallel
Rds = 4.68e-3;          % Drain-source ON resistance
Rbus = 0.020;		% Total bus resistance 

% Estimate current and duty cycle required to deliver 2kW to load,
% based on bus resistance and Vg given above
power = 2000;
for i = 1:length(Vbus)
  I(i) = min(roots([-Rbus, Vbus(i), -power]));
  assert(isreal(I(i)) && I(i) > 0)

  temp = roots([Vbus(i)^2, -Vbus(i)*Vg, -power*Rm]);
  D(i) = min(temp(temp > 0 & temp < 1));
end

I = I
D = D

% P_ss = steady state power losses
% P_sw = switching power losses
P_ss = Rds.*(I./N).^2;
P_sw =  Vbus.*(I./N)*f*tr;

% ON-BRAKE mode: Q3 is always on; Q1 and Q2 are switching
% Assume feed transistor always on
P_q1 = P_ss .* D + P_sw;
P_q2 = P_ss .* (1-D) + P_sw;
P_q3 = P_ss;

% Largest amount of power in any MOSFET
P_max = max(P_q1, max(P_q2, P_q3))

% Total power that must be removed by the heatsink
Ptot = (P_q1 + P_q2 + P_q3 + P_ss)*N

subplot(3,2,1)
plot (Vbus, Ptot)
xlabel('Vbus (Volts)')
ylabel('Total power loss (Watts)')
grid on

subplot(3,2,2)
plot(Vbus, P_max)
xlabel('Vbus (Volts)')
ylabel('Maximum device power loss (Watts)')
grid on


% Thermal parameters
Tmax = 175 * 0.75;	% MOSFET maximum junction temperature
Ta = 45;		% Ambient temperature
DT = Tmax - Ta;		% 

% IRF4468 thermal resistance
% NOTE: 100 PSI may not be feasible, but the Sil-Pad A2000 resistance 
% is only 12% higher at 10 PSI, and 7% higher at 25 PSI. Assuming the
% CD-02-05 is the same, we can expect Rcs < 0.45. Unfortunately, the 
% CD-02-05 datasheet doesn't confirm this theory...
Rjc = 0.3       % Junction to case
%Rcs = 0.4;      % case-heatsink (CD-02-05 pad @ 100 PSI)
%Rcs = 1.15;     % case-heatsink (Sil-Pad A2000 @ 100 PSI)
Rcs = 0.45;      % case-heatsink (CD-02-05 pad @ 10 PSI [EXTRAPOLATED])
Rjs = Rjc + Rcs % Total junction-heatsink thermal resistance

% IXFN360N junction-heatsink thermal resistance (paste)
%Rjs = 0.25;

% Required R_thermal if each MOSFET is mounted on it's own heatsink
Rs0 = DT ./ P_max - Rjs

% Required R_thermal if all MOSFETs are mounted on common heatsink
Rsa = (DT - P_max * Rjs)./Ptot

subplot(3,2,3)
plot(Vbus, Rsa)
xlabel('Vbus (Volts)')
ylabel('Minimum heatsink thermal resistance (C/W)')
grid on

%%%%%%%%%

% By definition, Vm is always the same for a given Vg and power to load
Vm = Vbus(1)*D(1);
Irip_RMS = D.*(Vbus - Vm)/(f*L*sqrt(3))

subplot(3,2,4)
plot(Vbus, Irip_RMS)
xlabel('Vbus (Volts)')
ylabel('RMS Ripple current (A)')
grid on

subplot(3,2,5)
plot(Vbus, D)
xlabel('Vbus (Volts)')
ylabel('Duty cycle at max load')
grid on






pause()

