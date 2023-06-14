% Building and defining a MATLAB based PID controller using simple routines. 

clc;clear;close all
tic                 % start timer to calculate CPU time

sp = 1;             % set point
feed1 = 1;          % coeficiente B
feed2 = 1;          % coeficiente K

Kp = 1;             % termo proporcional (Kp)
Ki = 0.01;          % termo integral (Ki)
Kd = 0.01;          % termo derivativo (Kd)
dt = 0.01;          % intervalo de simulação
tf = 30;            % tempo final de simulação
n = round(tf/dt);   % número de passos

% inicialização com zero das variáveis
prop(1:n+1) = 0; der(1:n+1) = 0; int(1:n+1) = 0; I(1:n+1) = 0;
PID(1:n+1) = 0;
saida(1:n+1) = 0;
y(1:n+1) = 0;
erro(1:n+1) = 0;

s1(1:n+1) = 0; S1(1:n+1) = 0;
s2(1:n+1) = 0; S2(1:n+1) = 0;

for i = 1:n
    erro(i+1) = sp - saida(i); % cálculo do erro
    
    prop(i+1) = erro(i+1);% proporcional do erro
    der(i+1)  = (erro(i+1) - erro(i))/dt; % derivação do erro
    int(i+1)  = (erro(i+1) + erro(i))*dt/2; % integração do erro
    I(i+1)    = sum(int); % soma da integração do erro
    
    PID(i+1)  = Kp*prop(i) + Ki*I(i+1)+ Kd*der(i); % PID
    
    %% Váriaveis para simulação
    S1(i+1) = sum(PID); % soma dos termos do PID para calcular a primeira integração
    s2(i+1) = (S1(i+1) + S1(i))*dt/2; % saída após a primeira integração
    S2(i+1) = sum(s2); %  soma da saída da primeira integração para calcular a segunda integração
    y(i+1) = (S2(i+1) + S2(i))*dt/2; % saída após a segunda integração
    saida(i+1) = s2(i+1)*feed1 + y(i+1)*feed2;
end

tsim = toc % tempo de simulação

% plot results
T = 0:dt:Time;
Reference = desired*ones(1,i+1);
plot(T,Reference,'r', 'LineWidth', 1.6)
hold on
plot(T,Output,'b', 'LineWidth', 1.6)

% simulação
t = 0:dt:tf;
sp = sp*ones(1,i+1);
plot(t,sp,'r','linewidth', 1.6)
hold on
plot(t,y,'b', 'linewidth', 1.6)
xlabel('Tempo (s)')
legend('Set Point (SP)','Sistema')

