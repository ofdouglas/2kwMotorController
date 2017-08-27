function [p_est, p_err, v_int, v_est] = tracking_loop(p, Kp, Ki, Ti)
p_est = zeros(1, length(p));
p_err = zeros(1, length(p));
v_int = zeros(1, length(p));
v_est = zeros(1, length(p));

for i = 2:length(p)
    p_est(i) = p_est(i-1) + v_est(i-1) * Ti;
    p_err(i) = p(i) - p_est(i);
    v_int(i) = v_int(i-1) + p_err(i) * Ki * Ti;
    v_est(i) = p_err(i) * Kp + v_int(i);
end

