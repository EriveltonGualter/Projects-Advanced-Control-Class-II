% declarar as funções de ponderação
nw1 = [0.05 10];
dw1 = [5 1];

nw2 = [0];
dw2 = [0];

nw3 = [1 0];
dw3 = [0 100];

w1 = tf(nw1, dw1);
w2 = tf();
w3 = tf(nw3, dw3);

% declarar a planta
ng = 1;
dg = [1 1];

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
[numk, denk] = ss2tf(ak, bk, ck, dk)

run('block_diagram.slx');
