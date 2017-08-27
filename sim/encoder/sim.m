Kp = 8000.0;
Ki = 40.0;
h = get_pi_impulse(Kp, Ki);

t = 0:1e-3:1;
v_actual = [t, ones(1,length(t)), 1-t];
p_actual = pos_from_velocity(v_actual);

v_est = conv(p_actual, h) ./ Kp;
length(v_est)
length(v_actual)

tp = 1:1:length(v_actual);
%plot(tp, v_est(1:length(tp)), tp, v_actual);

subplot(3,1,1)
plot(v_est)

subplot(3,1,2)
plot(v_actual)

subplot(3,1,3)
plot(p_actual)


input('')







