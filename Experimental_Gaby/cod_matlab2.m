% ref 
ref_file = 'C:\Users\gabri\Documents\Bolsa_INCT\Painel_Solar\ref_valores\sun_curve_data.txt';
r = load(ref_file); 
N = length(r); % Número de amostras N = tamanho(ref)

% parametros 
tau = 1.5; 
ts = 0.1; 
taumf = 10; 

% parâmetros da função afim Ke(u) = Ke0 + Ke1*u
Ke0 = 0.61;
Ke1 = -0.103;

% inicialização
y = zeros(1, N); % saída
u = zeros(1, N); % entrada
e = zeros(1, N); % erro

% loop de simulação
for k = 2:N
    % erro
    e(k) = r(k) - y(k);
    
    % calcula Ke(u(k-1))
    ke = Ke0 + Ke1 * r(k);
    
    % saturação de ke pra evitar valores negativos ou zero
    if ke < 0.01
        ke = 0.01;
    end
    
    % calcula os ganhos adaptativos
    ki = (1 / ke) * taumf;
    kp = tau * ki;
    
    % controle PI com EAD2
    u(k) = u(k-1) + ((kp + ki * ts) * e(k)) - kp * e(k-1);
    
    % simulação da saída com EAD1
    if k < N
        y(k+1) = ((1 - (ts / tau)) * y(k)) + ((ts * ke / tau) * u(k));
    end
end

% gráficos
figure;
subplot(3, 1, 1);
plot(1:N, r, 'r--', 'LineWidth', 1.5); hold on;
plot(1:N, y, 'b-', 'LineWidth', 1.5);
xlabel('Amostras (k)');
ylabel('Saída (y)');
legend('Referência (r)', 'Saída (y)');
title('Saída vs Referência');
grid on;

subplot(3, 1, 2);
plot(1:N, e, 'g-', 'LineWidth', 1.5);
xlabel('Amostras (k)');
ylabel('Erro (e)');
title('Erro ao longo do tempo');
grid on;

subplot(3, 1, 3);
plot(1:N, u, 'm-', 'LineWidth', 1.5);
xlabel('Amostras (k)');
ylabel('Entrada (u)');
title('Entrada de controle (u)');
grid on;