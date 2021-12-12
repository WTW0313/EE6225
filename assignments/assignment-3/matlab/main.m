tsim = 200;
t = 1 : tsim;
[y1, y2, y3, u1, u2, u3, j1, j2, j3] = MPC(1, 20, 3, 0.01);
plot(t, y1, 'r', t, y2, 'y', t, y3, 'b');
legend('y1', 'y2', 'y3');

subplot(3, 3, 1);
plot(t, y1, '-r');
hold on;
ylabel('Y_1');
subplot(3, 3, 4);
stairs(t, u1, '-r');
hold on;
ylabel('U_1');
subplot(3, 3, 7);
stairs(t, j1, '-r');
hold on;
ylabel('J_1');

subplot(3, 3, 2);
plot(t, y2, '-r');
hold on;
ylabel('Y_2');
subplot(3, 3, 5);
stairs(t, u2, '-r');
hold on;
ylabel('U_2');
subplot(3, 3, 8);
stairs(t, j2, '-r');
hold on;
ylabel('J_2');

subplot(3, 3, 3);
plot(t, y3, '-r');
hold on;
ylabel('Y_3');
subplot(3, 3, 6);
stairs(t, u3, '-r');
hold on;
ylabel('U_3 ');
subplot(3, 3, 9);
stairs(t, j3, '-r');
hold on;
ylabel('J_3');

[y1, y2, y3, u1, u2, u3, j1, j2, j3] = MPC(1, 15, 2, 0.01);

subplot(3, 3, 1);
plot(t, y1, '--g');
hold on;
ylabel('Y_1');
subplot(3, 3, 4);
stairs(t, u1, '--g');
hold on;
ylabel('U_1');
subplot(3, 3, 7);
stairs(t, j1, '--g');
hold on;
ylabel('J_1');

subplot(3, 3, 2);
plot(t, y2, '--g');
hold on;
ylabel('Y_2');
subplot(3, 3, 5);
stairs(t, u2, '--g');
hold on;
ylabel('U_2');
subplot(3, 3, 8);
stairs(t, j2, '--g');
hold on;
ylabel('J_2');

subplot(3, 3, 3);
plot(t, y3, '--g');
hold on;
ylabel('Y_3');
subplot(3, 3, 6);
stairs(t, u3, '--g');
hold on;
ylabel('U_3 ');
subplot(3, 3, 9);
stairs(t, j3, '--g');
hold on;
ylabel('J_3');

[y1, y2, y3, u1, u2, u3, j1, j2, j3] = MPC(1, 20, 2, 0.01);

subplot(3, 3, 1);
plot(t, y1, ':b');
ylabel('Y_1');
subplot(3, 3, 4);
stairs(t, u1, ':b');
ylabel('U_1');
subplot(3, 3, 7);
stairs(t, j1, ':b');
ylabel('J_1');

subplot(3, 3, 2);
plot(t, y2, ':b');
ylabel('Y_2');
subplot(3, 3, 5);
stairs(t, u2, ':b');
ylabel('U_2');
subplot(3, 3, 8);
stairs(t, j2, ':b');
ylabel('J_2');

subplot(3, 3, 3);
plot(t, y3, ':b');
ylabel('Y_3');
subplot(3, 3, 6);
stairs(t, u3, ':b');
ylabel('U_3 ');
subplot(3, 3, 9);
stairs(t, j3, ':b');
ylabel('J_3');
