function h = get_pi_impulse(Kp, Ki)
Gc = tf(Kp) + tf(Ki, [1 0]);
%Gc = tf(Kp);
Gf = tf(1, [1 0]);
Gt = feedback(Gc, Gf);
h = impulse(Gc);
end
