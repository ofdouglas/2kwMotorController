% Simulation parameters
dT = 1e-3;                    % Timestep
Vmax = 1;                   % Peak velocity (revs / sec)
accel = 10;                   % Acceleration slope
N = 2400;                     % Encoder ticks per revolution
Kp = 250;                      % Tracking loop proportional gain
Ki = 1000;                     % Tracking loop integral gain


% Generate real velocity and position 
% v : revs / sec
% p : revs
t = 0:dT:1;
%v = [t, ones(1, 2*length(t)), (1-t)] * Vmax;
%[b, a] = butter(2, dT * 4);
%v = filter(b, a, v);

v = Vmax * sin(4*2*pi*t);
p = pos_from_velocity(v) * dT;

% Quantized position (encoder output)
p_quant = floor(p*N)/N; 
%p_quant = filter(b, a, p_quant);
[p_est, p_err, v_int, v_est] = tracking_loop(p_quant, Kp, Ki, dT);


subplot(2,2,1)
plot(p)
xlabel('Time (ms)')
ylabel('Position (revs)')
grid on

subplot(2,2,2)
plot(v)
xlabel('Time (ms)')
ylabel('Velocity (revs/sec)')
grid on

subplot(2,2,3)
plot(p_quant)
xlabel('Time (ms)')
ylabel('Quantized position (revs)')
grid on

subplot(2,2,4)
plot(v_est)
xlabel('Time (ms)')
ylabel('Velocity estimate (revs/sec)')
grid on

input('')

v_err = (v_est .- v)/Vmax;
subplot(1, 1, 1)
plot(v_err);

input('')