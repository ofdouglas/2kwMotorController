

% From (bash)  sim.c >data.txt
results = load('data.txt')

error_signal = results(:,1);
controller_output = results(:,2);
plant_output = results(:,3);


subplot(3,1,1)
plot(error_signal)
ylabel('error signal')
grid on

subplot(3,1,2)
plot(controller_output)
ylabel('controller output')
grid on

subplot(3,1,3)
plot(plant_output)
ylabel('plant output')
grid on

input('')