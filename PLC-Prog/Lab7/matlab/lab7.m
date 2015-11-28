%% Clean up workspace
clc;
clear;
close all;

%% Setup plots
duration       = 1.4;
x_min_off_0_90 = 1;
x_max_off_0_90 = x_min_off_0_90 + duration;
x_min_off_90_0 = 1.2;
x_max_off_90_0 = x_min_off_90_0 + duration;
x_min_on_0_90  = 1.4;
x_max_on_0_90  = x_min_on_0_90 + duration;
x_min_on_90_0  = 1.4;
x_max_on_90_0  = x_min_on_90_0 + duration;

y_min_r_y    =  -30;
y_max_r_y    =  120;
y_min_u_usat = -100;
y_max_u_usat =  100;
y_min_p_i_d  = -120;
y_max_p_i_d  =  120;

%% Import data
import_off_0_90_table;
import_off_90_0_table;
import_on_0_90_table;
import_on_90_0_table;

%% Plot data
figure(1);
plot(data_off_0_90.time, data_off_0_90.r, data_off_0_90.time, data_off_0_90.y);
axis([x_min_off_0_90 x_max_off_0_90 y_min_r_y y_max_r_y]);
title('Step response, ARW off, 0..90');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');
print('-dpdf', 'off_0_90_r_y');

figure(2);
plot(data_off_0_90.time, data_off_0_90.u, data_off_0_90.time, data_off_0_90.u_sat);
axis([x_min_off_0_90 x_max_off_0_90 y_min_u_usat y_max_u_usat]);
title('Step response, ARW off, 0..90');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');
print('-dpdf', 'off_0_90_u_usat');

figure(3);
plot(data_off_0_90.time, data_off_0_90.p, data_off_0_90.time, data_off_0_90.i, data_off_0_90.time, data_off_0_90.d);
axis([x_min_off_0_90 x_max_off_0_90 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW off, 0..90');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'off_0_90_p_i_d');

figure(4);
plot(data_off_90_0.time, data_off_90_0.r, data_off_90_0.time, data_off_90_0.y);
axis([x_min_off_90_0 x_max_off_90_0 y_min_r_y y_max_r_y]);
title('Step response, ARW off, 90..0');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');
print('-dpdf', 'off_90_0_r_y');

figure(5);
plot(data_off_90_0.time, data_off_90_0.u, data_off_90_0.time, data_off_90_0.u_sat);
axis([x_min_off_90_0 x_max_off_90_0 y_min_u_usat y_max_u_usat]);
title('Step response, ARW off, 90..0');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');
print('-dpdf', 'off_90_0_u_usat');

figure(6);
plot(data_off_90_0.time, data_off_90_0.p, data_off_90_0.time, data_off_90_0.i, data_off_90_0.time, data_off_90_0.d);
axis([x_min_off_90_0 x_max_off_90_0 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW off, 90..0');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'off_90_0_p_i_d');

figure(7);
plot(data_on_0_90.time, data_on_0_90.r, data_on_0_90.time, data_on_0_90.y);
axis([x_min_on_0_90 x_max_on_0_90 y_min_r_y y_max_r_y]);
title('Step response, ARW on, 0..90');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');
print('-dpdf', 'on_0_90_r_y');

figure(8);
plot(data_on_0_90.time, data_on_0_90.u, data_on_0_90.time, data_on_0_90.u_sat);
axis([x_min_on_0_90 x_max_on_0_90 y_min_u_usat y_max_u_usat]);
title('Step response, ARW on, 0..90');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');
print('-dpdf', 'on_0_90_u_usat');

figure(9);
plot(data_on_0_90.time, data_on_0_90.p, data_on_0_90.time, data_on_0_90.i, data_on_0_90.time, data_on_0_90.d);
axis([x_min_on_0_90 x_max_on_0_90 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW on, 0..90');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'on_0_90_p_i_d');

figure(10);
plot(data_on_90_0.time, data_on_90_0.r, data_on_90_0.time, data_on_90_0.y);
axis([x_min_on_90_0 x_max_on_90_0 y_min_r_y y_max_r_y]);
title('Step response, ARW on, 90..0');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');
print('-dpdf', 'on_90_0_r_y');

figure(11);
plot(data_on_90_0.time, data_on_90_0.u, data_on_90_0.time, data_on_90_0.u_sat);
axis([x_min_on_90_0 x_max_on_90_0 y_min_u_usat y_max_u_usat]);
title('Step response, ARW on, 90..0');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');
print('-dpdf', 'on_90_0_u_usat');

figure(12);
plot(data_on_90_0.time, data_on_90_0.p, data_on_90_0.time, data_on_90_0.i, data_on_90_0.time, data_on_90_0.d);
axis([x_min_on_90_0 x_max_on_90_0 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW on, 90..0');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'on_90_0_p_i_d');

% With subplots
figure(13);
subplot(3,1,1);
plot(data_off_0_90.time, data_off_0_90.r, data_off_0_90.time, data_off_0_90.y);
axis([x_min_off_0_90 x_max_off_0_90 y_min_r_y y_max_r_y]);
title('Step response, ARW off, 0..90');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');

subplot(3,1,2);
plot(data_off_0_90.time, data_off_0_90.u, data_off_0_90.time, data_off_0_90.u_sat);
axis([x_min_off_0_90 x_max_off_0_90 y_min_u_usat y_max_u_usat]);
title('Step response, ARW off, 0..90');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');

subplot(3,1,3);
plot(data_off_0_90.time, data_off_0_90.p, data_off_0_90.time, data_off_0_90.i, data_off_0_90.time, data_off_0_90.d);
axis([x_min_off_0_90 x_max_off_0_90 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW off, 0..90');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'off_0_90');

figure(14);
subplot(3,1,1);
plot(data_off_90_0.time, data_off_90_0.r, data_off_90_0.time, data_off_90_0.y);
axis([x_min_off_90_0 x_max_off_90_0 y_min_r_y y_max_r_y]);
title('Step response, ARW off, 90..0');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');

subplot(3,1,2);
plot(data_off_90_0.time, data_off_90_0.u, data_off_90_0.time, data_off_90_0.u_sat);
axis([x_min_off_90_0 x_max_off_90_0 y_min_u_usat y_max_u_usat]);
title('Step response, ARW off, 90..0');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');

subplot(3,1,3);
plot(data_off_90_0.time, data_off_90_0.p, data_off_90_0.time, data_off_90_0.i, data_off_90_0.time, data_off_90_0.d);
axis([x_min_off_90_0 x_max_off_90_0 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW off, 90..0');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'off_90_0');

figure(15);
subplot(3,1,1);
plot(data_on_0_90.time, data_on_0_90.r, data_on_0_90.time, data_on_0_90.y);
axis([x_min_on_0_90 x_max_on_0_90 y_min_r_y y_max_r_y]);
title('Step response, ARW on, 0..90');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');

subplot(3,1,2);
plot(data_on_0_90.time, data_on_0_90.u, data_on_0_90.time, data_on_0_90.u_sat);
axis([x_min_on_0_90 x_max_on_0_90 y_min_u_usat y_max_u_usat]);
title('Step response, ARW on, 0..90');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');

subplot(3,1,3);
plot(data_on_0_90.time, data_on_0_90.p, data_on_0_90.time, data_on_0_90.i, data_on_0_90.time, data_on_0_90.d);
axis([x_min_on_0_90 x_max_on_0_90 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW on, 0..90');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'on_0_90');

figure(16);
subplot(3,1,1);
plot(data_on_90_0.time, data_on_90_0.r, data_on_90_0.time, data_on_90_0.y);
axis([x_min_on_90_0 x_max_on_90_0 y_min_r_y y_max_r_y]);
title('Step response, ARW on, 90..0');
legend('r', 'y');
xlabel('time [s]');
ylabel('r, y');

subplot(3,1,2);
plot(data_on_90_0.time, data_on_90_0.u, data_on_90_0.time, data_on_90_0.u_sat);
axis([x_min_on_90_0 x_max_on_90_0 y_min_u_usat y_max_u_usat]);
title('Step response, ARW on, 90..0');
legend('u', 'u_{sat}');
xlabel('time [s]');
ylabel('u, u_{sat}');

subplot(3,1,3);
plot(data_on_90_0.time, data_on_90_0.p, data_on_90_0.time, data_on_90_0.i, data_on_90_0.time, data_on_90_0.d);
axis([x_min_on_90_0 x_max_on_90_0 y_min_p_i_d y_max_p_i_d]);
title('Step response, ARW on, 90..0');
legend('p', 'i', 'd');
xlabel('time [s]');
ylabel('p, i, d');
print('-dpdf', 'on_90_0');

