import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d
from matplotlib.animation import FuncAnimation

# 您提供的點
points = np.array([[568, 481], [328, 270], [284, 380], [587, 284], [36, 260], [161, 373]])

# 建立 Voronoi 圖
vor = Voronoi(points)

# 畫圖
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111)

# 畫 Voronoi 圖（含點、線、自動延伸）
voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors='blue', line_width=2)

# 畫出原始點
ax.plot(points[:, 0], points[:, 1], 'ro')
for i, (x, y) in enumerate(points):
    ax.text(x + 0.1, y + 0.1, f'P{i+1}', color='red')

# 設定圖範圍
ax.set_xlim(0, 600)
ax.set_ylim(0, 600)
ax.set_title("Voronoi Diagram of 4 Points")

plt.grid(True)
plt.show()