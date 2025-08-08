import matplotlib.pyplot as plt

# 定義點的座標，按照 P1、P2、P3 的順序
x = [323.0, 478.0, 395.0]
y = [329.0, 369.0, 228.0]
labels = [1, 2, 3]

# 創建繪圖
plt.figure(figsize=(8, 6))
plt.scatter(x, y, color='blue', s=100, zorder=2, label='Points')

# 添加點的標註 (1, 2, 3)
for i, label in enumerate(labels):
    plt.text(x[i] + 5, y[i] + 5, str(label), fontsize=12, color='red', weight='bold')

# 連接點來表示順序
plt.plot(x, y, linestyle='--', color='gray', zorder=1, label='Sequence')

# 設定圖表標題和座標軸標籤
plt.title('Plot of Points P1, P2, and P3')
plt.xlabel('x-coordinate')
plt.ylabel('y-coordinate')

# 添加網格
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

# 儲存圖表為 PNG 檔案
plt.savefig('points_plot.png')
print("圖表已儲存為 'points_plot.png'")