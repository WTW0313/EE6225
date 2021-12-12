clc,clear;
%% import parameters
g11 = tf(-0.98,[12.5 1],'InputDelay',17);
g21 = tf(-0.43,[14.7 1],'InputDelay',25);
g31 = tf(-0.12,[15 1],'InputDelay',31);
g12 = tf(-0.36,[15 1],'InputDelay',27);
g22 = tf(-0.92,[13 1],'InputDelay',16);
g32 = tf(-0.16,[15 1],'InputDelay',34);
g13 = tf(-0.14,[15.2 1],'InputDelay',32);
g23 = tf(-0.11,[15.6 1],'InputDelay',33);
g33 = tf(-1.02,[11.8 1],'InputDelay',16);
Gs = [g11 g12 g13; g21 g22 g23; g31 g32 g33];
%% create MPC controller object with sample time
mpc1 = mpc(Gs, 1);
%% specify prediction horizon
mpc1.PredictionHorizon = 50; %N2
%% specify control horizon
mpc1.ControlHorizon = 10; %Nu
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = [0;0;0];
mpc1.Model.Nominal.Y = [0;0;0];
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -4;
mpc1.MV(1).Max = 4;
mpc1.MV(2).Min = -4;
mpc1.MV(2).Max = 4;
mpc1.MV(3).Min = -4;
mpc1.MV(3).Max = 4;
%% specify weights
mpc1.Weights.MV = [0 0 0];
mpc1.Weights.MVRate = [0.01 0.01 0.01];
mpc1.Weights.OV = [1 1 1];
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
mpc1_MDSignal=[];
mpc1_RefSignal=ones(201,3);
for i=1:50
    mpc1_RefSignal(i,2)=0;
    mpc1_RefSignal(i,3)=0;
    mpc1_RefSignal(i+50,3)=0;
end
sim(mpc1, 200, mpc1_RefSignal, mpc1_MDSignal, options);
