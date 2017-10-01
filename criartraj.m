button = 1;
axis([0 0.4 -0.3 0.3]) %eixos
figure(2)
grid on
hold on

rx = []; %vetor para receber coordenadas de x
ry = []; %vetor para receber coordenadas de y
rz = []; %vetor para receber coordenadas de x

%vai fazer a trajetoria come?ar da posi?? inicial
rx(1) = 0.23;
ry(1) = 0;
rz(1) = 0.08;
i = 2;

while(button ==1) % enquantos clicamos com botao esquerdo do mause
rz(i) = 0.07;
[x,y,button] = ginput(1); %obtemos a coordenada de um ponto da áres
plot3(x,y,rz,'ro'); %mostramos o ponto
%terminamos de ler com um clique no botão direito
rx(i) = x;
ry(i) = y;
i = i+1;
end

x = rx;
y = ry;
dz = rz;

%correcao da altura em z
theta = [0 0 0];
d = [0.08 0 0];
l = [0 0.13 0.1];
alpha = [-pi/2 0 0];
t=[0:1:1]';

z = dz - d(1); 

%pontos em x espaço de trabalho
NP = length(x); %numero de pontos
DIST =0.005; %distancia entre cada ponto

%TRAJETORIA
for i=1:1:(NP-1)
    distancia(i) = sqrt((x(i)-x(i+1))^2+(y(i)-y(i+1))^2+(z(i)-z(i+1))^2); % distancia entre cada ponto informado
    NPS(i) = distancia(i)/DIST;  %numero de pontos-1 por cada seção
    
    NPS(i) = round(NPS(i));
    xd(i) = (x(i+1)-x(i))/(NPS(i)); % distancia entre pontos no eixo x
    yd(i) = (y(i+1)-y(i))/(NPS(i)); % distancia entre pontos no eixo y
    zd(i) = (z(i+1)-z(i))/(NPS(i)); % distancia entre pontos no eixo z
    
    
    linhasyez=length(yd); 
    pos{1,i}=[]; %cria vetor da trajetoria em x
    pos{2,i}=[]; %cria vetor da trajetoria em y
    pos{3,i}=[]; %cria vetor da trajetoria em z
	for j=1:1:NPS(i)
        pos{1,i}=horzcat(pos{1,i},((x(i)-xd(i))+xd(1,linhasyez)*j));  
        pos{2,i}=horzcat(pos{2,i},((y(i)-yd(i))+yd(1,linhasyez)*j));
        pos{3,i}=horzcat(pos{3,i},((z(i)-zd(i))+zd(1,linhasyez)*j));
    end
    
%     pos{1,i}(1,NPS(i))=x(i+1); % ultimo valor de cada trajeto recebe o valor valor final desejado
%     pos{2,i}(1,length(pos{1,i}))=y(i+1); % ultimo valor de cada trajeto recebe o valor valor final desejado
%     pos{3,i}(1,length(pos{1,i}))=z(i+1); % ultimo valor de cada trajeto recebe o valor valor final desejado
end

CP = [];
trajx = [];
trajy = [];
trajz = [];

%separa variaveis de trajetoria
for i=1:1:(NP-1)
trajx = horzcat(trajx,pos{1,i});
trajy = horzcat(trajy,pos{2,i});
trajz = horzcat(trajz,pos{3,i});
end
NPN = length(trajx);
 
rad = pi/180;
graus = 180/pi;

%cinematica
    for i=1:1:NPN
    th0(i) = atan2(trajy(i),trajx(i));
    ix(i) = (trajx(i)^2 + trajy(i)^2)^0.5;
%     #stuff for calculating th2
    r_2(i) = ix(i)^2 + (-trajz(i))^2;
    l_sq = l(2)^2 + l(3)^2;
    term2 = (r_2(i) - l_sq)/(2*l(2)*l(3));
    term1 = ((1 - term2^2)^0.5)*-1;
%     #calculate th2
    th2(i) = atan2(term1, term2);
%     #optional line. Comment this one out if you 
%     #notice any problems
    th2(i) = -1*th2(i);
%   x =x y=-z z=y

%     #Stuff for calculating th2
    k1 = l(2) + l(3)*cos(th2(i));
    k2 = l(3)*sin(th2(i));
    r  = (k1^2 + k2^2)^0.5;
    gamma = atan2(k2,k1);
%     #calculate th1
    th1(i) = atan2(-trajz(i),ix(i)) - gamma;
    end

for i=1:1:NPN
    P{i} = [th0(i) th1(i) th2(i)]
end

CP = [];
for i=1:1:(NPN)
CP=vertcat(CP,P{i});
end

for i=1:1:(NPN-1)
    [q{i},qd{i},qdd{i}]=jtraj(P{i},P{i+1}, t); 
end

CQ = [];
for i=1:1:(NPN-1)
CQ = vertcat(CQ,q{i});
end

CQQ = [];
for i=1:1:(NPN-1)
CQQ = vertcat(CQQ,qd{i});
end

CQQQ = [];
for i=1:1:(NPN-1)
CQQQ = vertcat(CQQQ,qdd{i});
end

% figure(2)
% %unimos cada par de pontos
% robot.plot([0,0,0]);
% hold on
% plot3(rx,ry,rz);

%vetor tempo
npt = length(CQ); % numero de pontos da trajetoria
tempo_simulacao = 6; % tempo de simulação
freq = tempo_simulacao/npt; % frequencia de amostragem
i = 0:freq:(tempo_simulacao-freq); 
t = i;

%adequa matriz para simulink com tempo
t = t';
CQ = horzcat(t,CQ);
CQQ = horzcat(t,CQQ);
CQQQ = horzcat(t,CQQQ);

%mandar pra works
assignin('base','CQ',CQ);
assignin('base','CQQ',CQQ);
assignin('base','CQQQ',CQQQ);
% assignin('base','rx',rx);
% assignin('base','ry',ry);
% assignin('base','rz',rz);
