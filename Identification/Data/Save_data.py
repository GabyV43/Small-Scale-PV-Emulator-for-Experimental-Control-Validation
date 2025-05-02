import serial
import csv
import time
import os

# Configurações da porta serial
porta = 'COM5'  # Substitua pela porta serial do Arduino
baudrate = 9600  # Velocidade de comunicação

# Caminho onde o arquivo CSV será salvo
caminho_pasta = r'C:\Users\vitor\OneDrive\Documents\gaby_IC\Painel'
nome_arquivo = 'dados_painel.csv'
caminho_completo = os.path.join(caminho_pasta, nome_arquivo)

# Criar a pasta caso não exista
os.makedirs(caminho_pasta, exist_ok=True)

# Configurar conexão com Arduino
try:
    arduino = serial.Serial(porta, baudrate, timeout=1)
    print(f"Conectado à porta {porta}")
    time.sleep(2)  # Aguardar estabilização da conexão
except Exception as e:
    print(f"Erro ao conectar na porta {porta}: {e}")
    exit()

# Abrir arquivo para salvar os dados
with open(caminho_completo, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['tempo (s)', 'entrada (V)', 'saida (V)'])  # Cabeçalhos do CSV
    
    print("Iniciando coleta de dados. Pressione Ctrl+C para interromper.")

    # Variáveis para medir tempo
    tempo_inicial = time.time()

    try:
        while True:
            linha = arduino.readline().decode('utf-8').strip()  # Ler dados da serial
            if linha:
                # Parsear os dados recebidos no formato: entrada,saida
                dados = linha.split(',')
                if len(dados) == 2:
                    entrada = float(dados[0])
                    saida = float(dados[1])
                    tempo_atual = time.time() - tempo_inicial

                    # Salvar no arquivo CSV
                    writer.writerow([tempo_atual, entrada, saida])
                    print(f"Tempo: {tempo_atual:.2f}s, Entrada: {entrada}V, Saída: {saida}V")

    except KeyboardInterrupt:
        print("\nColeta de dados interrompida pelo usuário.")

# Fechar conexão serial
arduino.close()
print(f"Dados salvos em '{caminho_completo}'.")
