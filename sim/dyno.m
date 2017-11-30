R1 = 0.089;
R2 = 0.156;
%R2 = 0.15;
I2 = 5:5:50;
N = length(I2);
eta = 0.8;

I1 = zeros(1,N);
Vemf = zeros(1,N);
V1 = zeros(1,N);
P1 = zeros(1,N);
P2 = zeros(1,N);

for i = 1:1:length(I2)
  I1(i) = max(roots([R1, (I2(i)*R2), -(I2(i)^2*R2/eta)]));
  Vemf(i) = I2(i) * R2;
  P1(i) = I1(i).^2*R1 + I1(i)*Vemf(i);
  V1(i) = I1(i)*R1 + Vemf(i);
  P2(i) = Vemf(i) * I2(i);
end

ax = plotyy(I2, V1, I2, I1);
xlabel("M2 Current (A)")
ylabel(ax(1), "M1 Drive Voltage (V)")
%ylabel(ax(1), "Back EMF (V)")
ylabel(ax(2), "M1 Current (A)")
grid on

V1
I1
I2
P2

% Achievable on 12V supply: [I1, I2, P2]
%  0.45 ohms: 26, 20, 180
%  0.3  ohms: 34, 28, 235
%  0.25 ohms: 37, 32, 256
%  0.2  ohms: 45, 40, 320
%  0.15 ohms: 45, 44, 290
%
%
%

% Efficiency:
%  Assume chain drive is 100% efficient
%  Assume gears are 90% efficienct
%  E30-400 @ 45A is about 76% efficient
%  
%
