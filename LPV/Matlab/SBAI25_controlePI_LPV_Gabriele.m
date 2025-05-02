%--------------------------------------------------------------------------
%% Controle PI LPV
% Marcelo MENEZES MORATO  
% 29/11/2024
%--------------------------------------------------------------------------
yalmip('clear');
clear all;
close all;
clc;

%% Parâmetros do modelo
% Processo identificado
% tau(p)dy + y = Ke(p)u

%Variação dos parâmetos (por enquanto, regra afim)
%tau(p) = tau0 + tau1*p
%Ke(p)  = Ke0 + Ke1*p
%Lei de adaptacao ==> p = 0.1*ref

tau0 = 1.4627;
tau1 = 0.02402;
Ke0  = 0.3624;
Ke1  = -0.05229;

% Taxa de variação do parâmetro p é limitada, dp = dp/dt
% p  \in [pmin,pmax]
% dp \in [dpmin,dpmax]

pmin  = 0.0;
pmax  = 1.9998;
dpmin = -0.079992;
dpmax =  0.079992;

% Lei de controle PI adaptiva/LPV
% e(t)  = r(t) - y(t)
% u(t)  = Kp(p)*e(t) + Ki(p)*IE(t)
% IE(t) = integral ( e(t) )

% Modelo estruturado do processo

%x     = [e , IE]'
%w     = [r , dr/dt]'
%dx/dt = A(p)x + B(p)u + Br(p)w

%u     = [Kp(p) Ki(p)]*x = K(p)*x

%A(p)  = [-1/tau(p) 0; 1 0];
%B(p)  = [-Ke(p)/tau(p); 0];
%Br(p) = [-1/tau(p) 1; 0 0];

% Restricoes do processo
nx = 2;
nw = 2;
nu = 1;

xmin = [-2 NaN]';
xmax = [2 NaN]';

umin = 0;
umax = 5;

% Restricao temporal ct de tempo de MF < 1/beta ==> t5% < 1.5 ==>  beta > 2
beta = 2; 

%% Síntese do controle PI LPV
% K(p) = K0 + K1*p
% P(p) = P0 + P1*p

% Troca de variáveis
% Y(p) = inv(P(p))
% W(p) = K(p)Y(p)  ==> K(p) = W(p)P(p)

Y0 = sdpvar(nx,nx,'symmetric');  
Y1 = sdpvar(nx,nx,'symmetric');  

W0 = sdpvar(nu,nx,'full');          
W1 = sdpvar(nu,nx,'full');          

gam_hinf = sdpvar(1,1,'full'); 

% Strictness 
eps = 1e-5;
% Constraints
F = [];
F = [F, Y0 >= 0];            %Y0 is Positive Def.
F = [F, Y1 >= 0];            %Y1 is Positive Def.
F = [F,gam_hinf >= 0];

for rho=pmin:0.1:pmax 
    
    tau = tau0 + tau1*rho;
    Ke  = Ke0 + Ke1*rho;

    Yrho = Y0 + Y1*rho;
    Wrho = W0 + W1*rho;
    F = [F, Yrho >= 0];    

    Arho  = [-1/tau 0; 1 0];
    Brho  = [-Ke/tau; 0];
    Brrho = [-1/tau 1; 0 0];
    
    %LMI1  --> Alocacao de "polos"
    % Api(p) = (A(p)+B(p)K(p)
    %Queremos 2*beta*Prho + Prho*Apirho + Apirho'*Prho + (***) <0;

    %(***) representa sum dP/drho <= dY/drho <= Y1*rhomax 
    %Logo:
    M11 = (Arho*Yrho + Brho*Wrho) + (Arho*Yrho + Brho*Wrho)' + Y1*dpmax;
    M12 = Yrho;
    M21 = M12';
    M22 = -Yrho*(1/(2*beta));
    
    LMI1 = [M11 M12; M21 M22];
    F    = [F, LMI1<= eps];
    F    = [F, -trace(Yrho) <= eps];

    %LMI2  --> Norma Hinf
    %Queremos minimizar a influcencia (energia-->enegia)
    %entre a "perturbacao" w e os estados estruturados x
    %Logo minimizamos a norma Hinf entre w-->x (cf. Bounded Real Lemma)

    M11 = (Arho*Yrho + Brho*Wrho) + (Arho*Yrho + Brho*Wrho)' + Y1*dpmax;
    M12 = Yrho*Brrho; 
    M13 = eye(nx);

    M21 = M12';
    M22 = -gam_hinf*eye(nw);
    M23 = zeros(nx,nw)';
    
    M31 = M13';
    M32 = M23';
    M33 = -gam_hinf*eye(nx);
    
    LMI2 = [M11 M12 M13;
            M21 M22 M23;
            M31 M32 M33];

    F    = [F,LMI2 <= eps];

    % LMI 3 --> Restricao em u (saturacao)
    % x'Px <= 1 ==> | hi u | <= umax, for h1,h2,...,hz
    
    M11 = (umax)^2;
    M12 = (Wrho);
    M22 = Yrho;
  
    LMI3 = [M11 M12; M12' M22];
    F    = [F,LMI3 >= eps];

    % LMI 4 --> Restricao em x1 (erro maximo)
    % x'Px <= 1 ==> | hi x | <= xmax, for h1,h2,...,hz
  
    His = eye(nx);
    hi = His(1,:);
    M11 = (xmax(1))^2;
    M12 = (hi*Yrho);
    M22 = Yrho;
    
    LMI4 = [M11 M12; M12' M22];
    F    = [F,LMI4 >= eps];
end


%SOLUTIONS
options = sdpsettings('solver', 'sdpt3','sdpt3.maxit',100);
%options = sdpsettings('solver', 'lmilab');%,'sdpt3.maxit',100);
options.savesolveroutput = 1;
%solution = optimize(F,rtrace,options);  
solution = optimize(F,gam_hinf,options);  

solution

%solutionmin = solvesdp(F,gammahinf,options);  
Y0 = double(Y0);
Y1 = double(Y1);
W0 = double(W0);
W1 = double(W1);

%% Simulação exemplo da MF
ref_file  = 'C:\Users\gabri\Documents\Bolsa_INCT\Painel_Solar\ref_valores\sun_curve_data_b.txt';
r = load(ref_file); 

npts = length(r);
%npts = tsim/Ts;

Ts   = 0.01;
tsim = npts*Ts;      %segundos
tpl  = 0:Ts:tsim-Ts;

y  = zeros(1,npts);
x  = zeros(2,npts);
uc = zeros(1,npts);
u  = zeros(1,npts);
e  = zeros(1,npts);
ie = zeros(1,npts);
%ref_file  = 'C:\Users\gabri\Documents\Bolsa_INCT\Painel_Solar\ref_valores\sun_curve_data.txt';
%r = load(ref_file); 
%r  = r + 0.25*cos(.3*tpl);
%dr = -0.25*0.3*sin(.3*tpl);

for k = 1:1:npts-1

    % Calculo do agendamento e das constantes referentes ao ponto de op.
    rho = 1*r(k);
    tau = tau0 + tau1*rho;
    Ke  = Ke0 + Ke1*rho;

    % Calculo dos erro de seguimento e sua integral
    e(k)   = r(k) - y(k);
    if k>1
        ie(k)  = ie(k-1) + Ts*e(k);
    else
    end

    % Calculo do estado estruturado
    x(:,k)  = [e(k) ie(k)]';

    % Calculo dos ganhos LPV
    Prho = inv(Y0 + rho*Y1);
    Wrho = W0 + rho*W1;
    Krho = Wrho*Prho;
    
    % Calculo controle PI LPV
    uc(k)   = Krho*x(:,k);
    % Calculo do u real, saturado
    u(k) = min(umax,max(umin,uc(k)));
   
    
    y(k+1) = y(k) - (Ts/tau)*y(k) + (Ts/tau)*Ke*u(k);
end

%% Plots
figure(1);
subplot(3,1,1);
grid on; hold on;
plot(tpl,r,'LineWidth',2,'color','black','LineStyle','-');
plot(tpl,y,'LineWidth',2,'color','red','LineStyle','-.');
legend('Reference','Output (Voltage)')
subplot(3,1,2);
grid on; hold on;
plot(tpl,uc,'LineWidth',2,'color','black','LineStyle','-');
%plot(tpl,u,'LineWidth',2,'color','cyan','LineStyle','--');
legend('Calculated control signal','entrada real, saturada (tensao)')
%subplot(3,1,3);
%grid on; hold on;
%plot(tpl,e,'LineWidth',2,'color','black','LineStyle','-');
%legend('erro de seguimento de referencia');
%% THE END