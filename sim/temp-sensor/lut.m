

# NXRT15WB473FA1B030 Thermistor parameters:
# B value is B25/100
R0 = 47e3;
Kc = 237.15;
T0 = 25 + Kc;
B = 4131;

R = 100:100:122300;

Rinf = R0 * e^(-B/T0);
Tc = B ./ log(R/Rinf) - Kc;

f = fopen('table.txt', 'w+');
fprintf(f, 'float thermistor_table[] = {\n')
for i = 1:1:length(Tc)-1
  fprintf(f, '\t%f, \t//R = %d\n', Tc(i), R(i))
end
fprintf(f, '\t%f\n', Tc(end))
fprintf(f, '};')
fclose(f);




