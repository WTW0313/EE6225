% Run the simulink model
sim('relay_feedback_simulink');
plot(out);

% Define the original transfer function
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));

% Determine parameters from the output plot
L = 2.4415;
Pu = 7.9145;
Kp = 0.1875;
h = 1;
a = 0.1815;

Wu = 2 * pi / Pu;
Ku = 4 * h / (a * pi);
T = sqrt((Kp * Ku)^2 - 1) / Wu;

Gp = Kp * exp(- L * s) / (T * s + 1);
step(G0, Gp);
legend('Original','Frist order plus time delay model');
title('Relay Feedback');
subtitle({'Frist order plus time delay model parameters', ['K =' num2str(K), ', T =', num2str(T), ', L =', num2str(L)]});

figure(2);
nyquist(G0, Gp, {0.0001, 0.9});
legend('Original','Frist order plus time delay model');