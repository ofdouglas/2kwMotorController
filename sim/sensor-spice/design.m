# Using VDDA=3.3V, assume op-amp saturates 50mV from rails
Vo_max = 3.25
Vo_min = 0.05

# Max design current is 90A, but overload up to 100 can be sensed
# Sensor output current is 1/2000 of primary current, bi-directional
Is_max = 0.045
Is_min = -0.045

# ADC input must be positive, so we'll shift the input such that
# Is_min corresponds with Vo_min, Is_max with Vo_max
Vo_shift = (Vo_max + Vo_min) / 2

# Now choose R so that (Is_max * R)/2 + Vo_shift == Vo_max
# (output of resistive level shifter is (Va + Vb)/2)
# R = 2 * (Vo_max - Vo_shift) / Is_max
R = 71.5	# Rounded to E48 value

# Antialising filter bandwidth = 80kHz to pick up a few harmonics
# of motor ripple current (f_pwm = 20kHz)
fc = 80e3
C = 1/(2*pi*fc*R)

# Volts per ampere (of primary current; Ip = 2000 * Is)
# VpA = (Vo_max - Vo_min) / (2000 * (Is_max - Is_min))
#
# Vsense = Is*R, but Ip = 2000*Is, and Vout = (Vsense + Vo_shift)/2
VpA = (R / 2000) / 2

# Amperes per volt (primary current from sensor output)
# You must remember to subtract Vo_shift from the sensor output
# before multiplying by this factor!
ApV = 1 / VpA

# Ex: Vo_shift = 1.65, Vout = 2.5, so
# Ip = (Vout - Vo_shift) * ApV = 47.8 amps




# TM4C1294KCPDT SAR ADC sampling time (can be multiplied by 2^N, N <= 7)
# Rs_max (max. resistance seen by ADC input) is proportional to t_acq.
t_acq = 125e-9;
Rs_max = 250;

# TM4C1294KCPDT SAR ADC input impedances
r_adc = 2.5e3;
c_adc = 10e-12;

# Size of reservoir cap for ADC input, allowing 5% droop (5% of charge for c_adc  
# will come from the buffer op-amp, the rest from the reservoir cap, c_flt)
q_sh = Vo_max * c_adc;
c_flt = q_sh / (0.05 * Vo_max)

# k = Num time constants for ADC input circuit to settle to 0.5 LSB error
# Maximum r_flt would satisfy k * Tau_flt <= t_acq; choose r_flt = 1/2 r_flt max
k = 9;
r_flt = (t_acq / (c_flt * k )) / 2

# Find the pole that c_flt / r_flt adds to the amp loop gain, for stability check
p_flt = 1 / (r_flt * c_flt)

# Now pick an op amp that has good phase margin when p_flt is added, low z_out,
# rail-to-rail input / output, good PSRR and CMRR, and low input bias current
# since we only need 80 kHz bandwidth (and our primary concern is frequencies up
# to a few kHz [motor electrical time constant]), high bandwidth / slew rate is
# not necessary.
