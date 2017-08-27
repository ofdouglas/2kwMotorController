rpm = 6000
ppr = 2400
f_timer = 120e6

rps = rpm / 60
f_pulse = ppr * rps

counts_per_pulse = f_timer / f_pulse
timer_error = 1 / counts_per_pulse
