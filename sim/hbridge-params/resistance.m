

function resistance()

rho = 1.68e-8;   # Resistivity of copper (ohm-meters)
Tc = 0.003862;  # Tempco of copper
DT = 60;       # Assume bus bar is at 60 C


# Total resistance of bus bar
l_bus = 0.15;
A_bus = (1e-3 * 4e-3);
R_bus = rho * (1 + Tc * DT) * l_bus / A_bus

# Single lead resistance of one TO-247 package, mounted down to stand
# NOTE: The bond wire resistance is included in the datasheet Rds_on
L_pin = 4.3e-3;
W_pin = 1.65e-3;
T_pin = 0.38e-3;
R_pin = rho * (1 + Tc * DT) * L_pin / (W_pin * T_pin)

# Total H-bridge resistance (bus plus 4 pins: D->S, D->S)
R_bridge = R_bus + 4 * R_pin
