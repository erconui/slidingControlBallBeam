clear;
clc;

% initial conditions
P.z0 = 0.25;
P.zdot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;

% sample rate of controller
P.Ts = 0.01;


P.m = .007;
P.g = 9.81;
P.L = .6;
P.M = .15;
P.Rm = 9;
P.Jm = 7.35 * 10^-4;
P.Km = .0075;
P.Kg = 75;
P.d = .03;
P.J1 = .001;
P.Kb = .5625;

P.k1 = P.Rm*P.Jm*P.L/(P.Km*P.Kg*P.d) + P.J1;
P.k2 = (P.L/P.d)*(P.Km*P.Kb/P.Rm + P.Kb + (P.Rm*P.Jm/(P.Km*P.Kg)));
P.k3 = 1 + P.Km/P.Rm;
P.k4 = 7/5;