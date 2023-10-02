% First we had to define our motor parameters, and tweak Kp to get a good
% result.
K=1.6; % DC gain [rad/Vs]
sigma=10; % time constant reciprocal [1/s]
Kp = 50;
open_system('motor_control.slx') %This opens the .slx file.
out=sim('motor_control');
% Copied and pasted into data workspace.
% I needed to manually set that the voltage and velocity were zero before
% 1 second, since I set our Serial Monitor to not output anything if the
% wheels weren't spinning for our convenience.
time = data(:,1); % These lines interpret the data into usable variable arrays.
time = time / 1000;
voltage = data(:,2);
rads = data(:,3);
pos = data(:,4);
pos = (pos * (pi/2) / 800);
% Finally, we had to plot the results
figure;
subplot(2,2,1);
plot(out.Voltage,'linewidth',2, 'Color', "#4DBEEE"); % Here we make a plot of the Voltage vs. Time
hold on;
xlabel('Time (s)');
ylabel('Voltage (V)');
plot(time,voltage, 'linewidth',2, 'Color', 'red');
xlim([0 3]);
legend('Simulated Voltage','Experimental Voltage');

subplot(2,2,2);
plot(out.DesiredVelocity,'linewidth',2, 'Color', "#4DBEEE"); % Here we make a plot of the Voltage vs. Time
hold on;
xlabel('Time (s)');
ylabel('Velocity (V)');
plot(time,rads, 'linewidth',2, 'Color', 'red');
xlim([0 3]);
legend('Simulated Velocity','Experimental Velocity');

subplot(2,2,3:4);
xlabel('Time (s)');
ylabel('Angular Positon (rad)');
hold on;
plot(out.DesiredPosition,'--','linewidth',3, 'Color','black');
plot(time,pos, 'linewidth',2,'Color','red');
plot(out.Position,'linewidth',2,'Color','#4DBEEE');
xlim([0 3]);
legend('Desired Position','Experimental Position','Simulated Position');