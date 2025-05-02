import os
import numpy as np
import matplotlib.pyplot as plt

directory = r'C:\Users\gabri\Documents\Bolsa_INCT\Painel_Solar\ref_valores'
file_name = "sun_curve_data.txt"
file_path = os.path.join(directory, file_name)

# delimitações
total_time = 100  
time_interval = 1  
peak_voltage = 2 
time = np.arange(0, total_time, time_interval)  

# Simulação da curva solar (parábola invertida)
scaled_time = (time - time.min()) / (time.max() - time.min())  
sun_curve = 4 * scaled_time * (1 - scaled_time)  
sun_curve = sun_curve * peak_voltage  # pico de 2V


# Por que usar random? Mesmo que você escolha o nível de nuvens (por exemplo, "pouco", "médio" ou "totalmente nublado"), dentro desse nível há uma variação natural e aleatória. Na vida real, as nuvens não criam um efeito constante — ora elas cobrem mais o sol, ora menos. O random é usado para imitar essa flutuação.
# Por que fixar com seed? A reprodutibilidade é útil se você quiser testar a mesma configuração várias vezes (como comparar os resultados para "nenhum" e "médio") e garantir que o comportamento aleatório seja consistente entre os testes. É como congelar o acaso para garantir que os mesmos dados sejam usados para validação.

def apply_clouds(voltage_curve, cloud_level):
    """Aplica diferentes níveis de nuvens como ruído na curva solar."""
    np.random.seed(42)  
    noise_levels = {"nenhum": 0, "pouco": 0.1, "medio": 0.3, "totalmente nublado": 0.5}
    noise = noise_levels.get(cloud_level, 0) * np.random.normal(0, 1, len(voltage_curve))
    return voltage_curve + noise


cloud_level = "nenhum"  # "nenhum", "pouco", "medio", "totalmente nublado"
adjusted_curve = apply_clouds(sun_curve, cloud_level)


if not os.path.exists(directory):
    os.makedirs(directory)
if not os.path.isfile(file_path):
    np.savetxt(file_path, adjusted_curve, delimiter=",")
    print(f"Arquivo criado com sucesso: {file_path}")
else:
    print(f"O arquivo já existe: {file_path}")

plt.figure(figsize=(10, 6))
plt.plot(time, adjusted_curve, label=f"Curva com nível de nuvens: {cloud_level.capitalize()}")
plt.title("Simulação de Tensão ao Longo do Dia")
plt.xlabel("Tempo (s)")
plt.ylabel("Tensão (V)")
plt.ylim(0, peak_voltage + 0.5)
plt.grid(True)
plt.legend()
plt.show()
