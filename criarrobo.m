clear all;
% close all;
clc

%constantes do compensador
conskp = 5;
conski = 2;
conskd = 1;
aggkp = 50;
aggki = 63;
aggkd = 2;

%setar parametros nos blocos PID
set_param('dinamica/integrador/KI1','Gain','2');
set_param('dinamica/integrador/KI2','Gain','2');
set_param('dinamica/integrador/KI3','Gain','2');
set_param('dinamica/proporcional/KP1','Gain','6');
set_param('dinamica/proporcional/KP2','Gain','6');
set_param('dinamica/proporcional/KP3','Gain','6');
set_param('dinamica/derivador/KD1','Gain','1');
set_param('dinamica/derivador/KD2','Gain','1');
set_param('dinamica/derivador/KD3','Gain','1');

%parametros motor
motor_R = 1.5;
motor_L= 0.006;
motor_K = 10/10000;
motor_J = 2/10000000;
motor_B = 9/10000000;

%parametros DH
theta = [0 0 0];
d = [0.08 0 0];
l = [0 0.13 0.1];
alpha = [-pi/2 0 0];

%criar elos
% theta | D | l | alpha | sigma | m | rx ry rz | Ixx Iyy Izz Ixy Iyz Ixz | Jm | G | B | Tc[0 0]   
 L(1)=Link([theta(1) d(1) l(1) alpha(1) 0 0.01 0.05 0.02 0.01 0 0 0 0 0 0 0 motor_J 1/50 motor_B 0 0]); 
 L(2)=Link([theta(2) d(2) l(2) alpha(2) 0 0.01 0.05 0.02 0.01 0 0 0 0 0 0 0 motor_J 1/50 motor_B 0 0]); 
 L(3)=Link([theta(3) d(3) l(3) alpha(3) 0 0.01 0.05 0.02 0.01 0 0 0 0 0 0 0 motor_J 1/50 motor_B 0 0]); 

%criar robo
robot = SerialLink(L);
assignin('base','robot',robot);
t=[0:1:1]';