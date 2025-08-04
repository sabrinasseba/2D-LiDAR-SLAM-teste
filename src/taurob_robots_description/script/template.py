import matplotlib as mpl
mpl.use('pgf')
import matplotlib.pyplot as plt
import numpy as np
import rosbag

# === Função para setar tamanho do gráfico em LaTeX ===
def set_size(width_pt, fraction=1, subplots=(1, 1)):
    """Set figure dimensions to sit nicely in our document."""
    fig_width_pt = width_pt * fraction
    inches_per_pt = 1 / 72.27
    golden_ratio = 0.8  # proporcionalmente mais compacto
    fig_width_in = fig_width_pt * inches_per_pt
    fig_height_in = fig_width_in * golden_ratio * (subplots[0] / subplots[1])
    return (fig_width_in, fig_height_in)

# === Configurações de estilo ===
# plt.style.use('seaborn-v0_8')  # comentado pois pode não existir
plt.style.use('ggplot')
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.rc('pgf', rcfonts=False)

# === Função para extrair primeiro scan do tópico /scan de um bagfile ===
def get_first_scan_ranges(bagfile_path):
    bag = rosbag.Bag(bagfile_path)
    scan_ranges = None
    angle_min = None
    angle_increment = None
    for topic, msg, t in bag.read_messages(topics=['/scan']):
        scan_ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        break
    bag.close()
    return scan_ranges, angle_min, angle_increment

# === Caminhos dos bagfiles ===
bags = {
    "Ground Truth": "ground_truth.bag",
    "Mapa Sem Filtro": "mapa_sem_filtro.bag",
    "Com Filtro e Parâmetros": "mapa_com_parametros_filtro.bag",
    "Com Filtro Sem Parâmetros": "mapa_sem_parametros_filtro.bag"
}

# === Dicionário para armazenar dados ===
scans_data = {}

# === Lê os dados de cada bag ===
for label, path in bags.items():
    ranges, angle_min, angle_increment = get_first_scan_ranges(path)
    scans_data[label] = ranges

# === Calcula erro médio absoluto em relação ao Ground Truth ===
ground_truth = scans_data["Ground Truth"]

errors = {}
for label, ranges in scans_data.items():
    if label == "Ground Truth":
        continue
    # Calcula o erro absoluto médio ignorando NaNs (se existirem)
    error = np.nanmean(np.abs(ranges - ground_truth))
    errors[label] = error

# === Gráfico de barras ===
fig, ax = plt.subplots(figsize=set_size(300))  # tamanho menor para gráfico de barras

bars_labels = list(errors.keys())
bars_values = [errors[label] for label in bars_labels]

ax.bar(bars_labels, bars_values, color=['orange', 'green', 'red'])
ax.set_ylabel(r'Erro Médio Absoluto (m)')
ax.set_title(r'Comparação de Acurácia entre Mapas')
ax.grid(axis='y')

# === Exportação ===
plt.tight_layout()
plt.savefig('plots/scan_profile_comparison.pdf', format='pdf', dpi=300, bbox_inches='tight')
plt.savefig('plots/scan_profile_comparison.pgf', format='pgf', bbox_inches='tight', facecolor='white', edgecolor='none')
