% declarar as funções de ponderação

s = tf('s');

w1 = 100*(0.005*s + 1)^2 / (0.2*s+1)^2
w2 = tf();
w3 = s^2 / 40000;

% Teste da Restrição:
% Magnitude e Fase das funções de ponderação
w = logspace(0,3);
[MAG_w1,PHASE_w1] = bode(w1, w);
[MAG_w3,PHASE_w3] = bode(w3, w);

% soma_mag = [];
% for i=1:length(MAG_w1)
%     soma_mag(i) = 20*log( 1/abs(MAG_w1(i)) + 1/abs(MAG_w3(i)) );
% end
% semilogx(w, soma_mag);
%     grid
%     title('Restrição para funções de ponderação');
%     xlabel('w [rad/s]');
%     ylabel('[1/db]');

% declarar a planta
ng = 400;
dg = [1 2 400];

% planta na representação espaço de estado
[AG, BG, CG, DG] = tf2ss(ng, dg);

% cria um sistema sysg
sysg = mksys(AG, BG, CG, DG);

% aumentar a planta
P = augtf(sysg, w1, w2, w3);

% calcular o controlador via hinf 
[ssk, sstyu] = hinf(P);    % ssk é o controlador na forma de espaço de estado

% a próxima linha vai extrair o modelo de estados de ssk
[ak, bk, ck, dk] = branch(ssk);

% controlador na forma de função de transferência
[numk, denk] = ss2tf(ak, bk, ck, dk);

contralador = tf(numk, denk)

run('model_with_disturbance.slx');
run('model_without_disturbance.slx');

s = zpk('s'); % Laplace variable s
MS = 2; AS = .03; WS = 5;
W1 = (s/MS+WS)/(s+AS*WS);
MT = 2; AT = .05; WT = 20;
W3 = (s+WT/MT)/(AT*s+WT);

[K1,CL1,GAM1] = mixsyn(tf(ng,dg),W1,[],W3);

L1 = tf(ng,dg)*K1;
I = eye(size(L1));
S1 = feedback(I,L1); % S=inv(I+L1);
T1 = I-S1;

step(T1,1.5);
title('\alpha and \theta command step responses');

figure;
sigma(I+L1,'--',T1,':',L1,'r--',W1/GAM1,'k--',GAM1/W3,'k-.',{.1,100})
legend('1/\sigma(S) performance','\sigma(T) robustness','\sigma(L) loopshape',...
       '\sigma(W1) performance bound','\sigma(1/W3) robustness bound')
