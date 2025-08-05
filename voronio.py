import tkinter as tk
import math
#Test
class Vertex:
    """代表一個點（或稱頂點）的類別"""
    def __init__(self, id, x=0.0, y=0.0):
        self.current_edges = []  # 儲存目前計算出來的中垂線
        self.id = id
        self.position = (x, y)
        self.incident_edge = None

    @property
    def x(self):
        return self.position[0]

    @property
    def y(self):
        return self.position[1]

    def __repr__(self):
        return f"Vertex({self.id}, {self.position})"

class Edge:
    """代表一條邊的類別"""
    def __init__(self, id, start_vertex, end_vertex, origin_vertices=None):
        self.id = id
        self.start = start_vertex
        self.end = end_vertex
        self.origin_vertices = origin_vertices  # 加上來源點資訊

    def __repr__(self):
        if self.origin_vertices:
            ov_str = f" from ({self.origin_vertices[0].id}, {self.origin_vertices[1].id})"
        else:
            ov_str = ""
        return f"Edge({self.id}, {self.start.id}->{self.end.id}){ov_str}"


class PointDrawerApp:
    def __init__(self, root, filename=None):
        self.root = root
        self.root.title("點座標顯示與測資載入")

        button_frame = tk.Frame(root)
        button_frame.pack(side=tk.TOP, pady=10)

        self.btn1 = tk.Button(button_frame, text="載入下一個測資", command=self.load_next_test_data)
        self.btn1.pack(side=tk.LEFT, padx=5)

        # 註：此處的 is_ccw=False 對應 sort_reverse=False，代表「順時針」
        self.btn2 = tk.Button(button_frame, text="計算 Voronoi (順時針)", command=lambda: self.run_less_three_voronoi(is_ccw=False))
        self.btn2.pack(side=tk.LEFT, padx=5)

        # 註：此處的 is_ccw=True 對應 sort_reverse=True，代表「逆時針」
        self.btn3 = tk.Button(button_frame, text="計算 Voronoi (逆時針)", command=lambda: self.run_less_three_voronoi(is_ccw=True))
        self.btn3.pack(side=tk.LEFT, padx=5)

        # 定義畫布的寬度和高度
        self.canvas_width = 600
        self.canvas_height = 600
        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        self.coord_label = tk.Label(root, text="座標: ")
        self.coord_label.pack(pady=10)

        self.clicked_point_id = None
        self.filename = filename
        self.test_data_blocks = []
        self.current_block_index = -1

        if self.filename:
            self.read_all_test_data()
            if self.test_data_blocks:
                self.load_next_test_data()

    def logical_to_tk_coords(self, x, y):
        """
        將邏輯座標 (左下為0,0, Y向上) 轉換為 Tkinter 畫布座標 (左上為0,0, Y向下)。
        """
        return x, self.canvas_height - y

    def tk_to_logical_coords(self, tk_x, tk_y):
        """
        將 Tkinter 畫布座標 (左上為0,0, Y向下) 轉換為邏輯座標 (左下為0,0, Y向上)。
        """
        return tk_x, self.canvas_height - tk_y

    def sort_vertices_by_angle(self, vertices, sort_reverse=False):
        """
        將頂點列表排序。
        - 新增：優先處理通用共線情況。
        - 對於非共線點，以幾何中心進行角度排序。

        :param vertices: Vertex 物件的列表。
        :param sort_reverse: 排序方向的旗標。False 為順時針, True 為逆時針。
        :return: 排序後的 Vertex 列表。
        """
        if len(vertices) < 2:
            return vertices

        # === 修改開始：新增通用共線性檢查 ===
        # 使用外積檢查三點是否共線 (處理浮點數誤差)
        if len(vertices) == 3:
            p1, p2, p3 = vertices[0], vertices[1], vertices[2]
            cross_product = (p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x)
            
            if abs(cross_product) < 1e-6:  # 若接近於 0，視為共線
                # 判斷線的主要走向 (水平或垂直)
                dx = max(v.x for v in vertices) - min(v.x for v in vertices)
                dy = max(v.y for v in vertices) - min(v.y for v in vertices)
                
                # 順時針 (sort_reverse=False) -> 預期為遞減排序 (reverse=True)
                # 逆時針 (sort_reverse=True) -> 預期為遞增排序 (reverse=False)
                # 因此，實際的 reverse 參數與 sort_reverse 相反
                actual_reverse = not sort_reverse

                if dx >= dy: # 如果線條比較水平，以 x 座標排序
                    return sorted(vertices, key=lambda v: v.x, reverse=actual_reverse)
                else: # 如果線條比較垂直，以 y 座標排序
                    return sorted(vertices, key=lambda v: v.y, reverse=actual_reverse)
        # === 修改結束 ===

        # 對於非共線的點，或點的數量不是3，使用原始的角度排序邏輯
        # (原始的水平/垂直檢查現在是通用共線檢查的子集，但保留以處理 > 3 個點的特殊情況)
        is_horizontal = all(v.y == vertices[0].y for v in vertices)
        if is_horizontal:
            return sorted(vertices, key=lambda v: v.x, reverse=sort_reverse)

        is_vertical = all(v.x == vertices[0].x for v in vertices)
        if is_vertical:
            return sorted(vertices, key=lambda v: v.y, reverse=not sort_reverse) # 垂直方向的順/逆時針定義與Y軸方向有關

        # 非共線情況：使用角度排序
        cx = sum(v.x for v in vertices) / len(vertices)
        cy = sum(v.y for v in vertices) / len(vertices)
        
        def angle_from_center(v):
            return math.atan2(v.y - cy, v.x - cx)
        
        # angle_from_center 預設是逆時針，所以 sort_reverse=True (逆時針) 時，sorted 的 reverse 應為 False
        sorted_vertices = sorted(vertices, key=angle_from_center, reverse=not sort_reverse)
        
        # 為了保持一致的起始點，將最右(下)的點作為起點
        max_x_index = max(range(len(sorted_vertices)), key=lambda i: (sorted_vertices[i].x, -sorted_vertices[i].y))
        
        return sorted_vertices[max_x_index:] + sorted_vertices[:max_x_index]

    def get_perpendicular_bisector(self, v1, v2, edge_id, length=1200):
        """計算兩點的中垂線，回傳一個 Edge 物件，包含來源點資訊"""
        mx = (v1.x + v2.x) / 2
        my = (v1.y + v2.y) / 2
        dx = v2.x - v1.x
        dy = v2.y - v1.y
        mag = (dx**2 + dy**2)**0.5
        if mag == 0: return None
        ndx, ndy = -dy / mag, dx / mag
        p1 = Vertex(-1, mx - ndx * (length / 2), my - ndy * (length / 2))
        p2 = Vertex(-2, mx + ndx * (length / 2), my + ndy * (length / 2))
        return Edge(edge_id, p1, p2, origin_vertices=(v1, v2))  # 將來源點傳入


    def get_line_intersection(self, edge1, edge2):
        """
        計算由兩個 Edge 物件所定義的兩條線的交點。
        :param edge1: 第一條線。
        :param edge2: 第二條線。
        :return: 一個代表交點的 Vertex 物件，如果兩線平行則回傳 None。
        """
        p1 = edge1.start
        p2 = edge1.end
        p3 = edge2.start
        p4 = edge2.end

        A1 = p2.y - p1.y
        B1 = p1.x - p2.x
        C1 = A1 * p1.x + B1 * p1.y

        A2 = p4.y - p3.y
        B2 = p3.x - p4.x
        C2 = A2 * p3.x + B2 * p3.y

        determinant = A1 * B2 - A2 * B1

        if abs(determinant) < 1e-9: # 處理浮點數誤差
            # 兩線平行
            return None
        else:
            x = (B2 * C1 - B1 * C2) / determinant
            y = (A1 * C2 - A2 * C1) / determinant
            return Vertex(id=-10, x=x, y=y) # 使用一個臨時ID

    def get_three_line_cross_point(self, v1, v2, v3):
        """
        找出三個頂點的中垂線交點（即三角形的外心）。
        :param v1, v2, v3: 三個 Vertex 物件。
        :return: 代表外心的 Vertex 物件，如果三點共線則回傳 None。
        """
        bisector1 = self.get_perpendicular_bisector(v1, v2, edge_id=-1)
        bisector2 = self.get_perpendicular_bisector(v2, v3, edge_id=-2)

        if bisector1 is None or bisector2 is None:
            return None

        intersection_point = self.get_line_intersection(bisector1, bisector2)
        return intersection_point

    def draw_edge_on_canvas(self, edge, color="red", width=1):
        """在畫布上畫一條邊，座標需轉換為 Tkinter 座標"""
        if not edge: return
        x1_tk, y1_tk = self.logical_to_tk_coords(edge.start.x, edge.start.y)
        x2_tk, y2_tk = self.logical_to_tk_coords(edge.end.x, edge.end.y)
        self.canvas.create_line(x1_tk, y1_tk, x2_tk, y2_tk, fill=color, width=width)

    def run_less_three_voronoi(self, is_ccw: bool):
        """
        整合後的函式：根據指定的順序計算並繪製 Voronoi 圖。
        """
        if self.current_block_index == -1 or not self.test_data_blocks:
            print("請先載入測資。")
            return

        points_to_process = self.test_data_blocks[self.current_block_index]
        vertices = [Vertex(id=i, x=p[0], y=p[1]) for i, p in enumerate(points_to_process)]

        if not 1 < len(vertices) <= 3:
            print("此函式只處理 2 或 3 個點的情況。")
            return

        title = "重新排序後的頂點 (逆時針)" if is_ccw else "重新排序後的頂點 (順時針)"
        sort_reverse_flag = is_ccw
        
        sorted_vertices = self.sort_vertices_by_angle(vertices, sort_reverse=sort_reverse_flag)
        
        self.canvas.delete("all")
        points_to_draw = self.test_data_blocks[self.current_block_index]
        for x, y in points_to_draw:
            self.draw_point(x, y, color="blue")
            tk_x, tk_y = self.logical_to_tk_coords(x, y)
            self.canvas.create_text(tk_x + 5, tk_y, text=f"({x},{y})", anchor="w", fill="black", font=("Arial", 9))


        print(f"--- {title} ---")
        for i, v in enumerate(sorted_vertices):
            order_text = str(i + 1)
            print(f"{order_text}: {v}")
            tk_vx, tk_vy = self.logical_to_tk_coords(v.x, v.y)
            self.canvas.create_text(tk_vx, tk_vy - 12, text=order_text, fill="purple", font=("Arial", 12, "bold"))
        print("-----------------------------------------")
        
        self.current_edges = []
        for i in range(len(sorted_vertices)):
            v1 = sorted_vertices[i]
            v2 = sorted_vertices[(i + 1) % len(sorted_vertices)]
            bisector = self.get_perpendicular_bisector(v1, v2, edge_id=i)
            if bisector:
                self.current_edges.append(bisector)

        if len(sorted_vertices) == 2:
            for edge in self.current_edges:
                self.draw_edge_on_canvas(edge, color="green", width=2)
            return

        if len(sorted_vertices) == 3:
            v1, v2, v3 = sorted_vertices[0], sorted_vertices[1], sorted_vertices[2]
            circumcenter = self.get_three_line_cross_point(v1, v2, v3)

            if not circumcenter:
                print("三點共線，繪製兩條平行的中垂線。")
                for i in range(len(self.current_edges) - 1):
                    edge = self.current_edges[i]
                    self.draw_edge_on_canvas(edge, color="green", width=2)
                return
            
            print(f"三線交點 (外心): ({circumcenter.x:.2f}, {circumcenter.y:.2f})")
            
            if 0 <= circumcenter.x <= self.canvas_width and \
               0 <= circumcenter.y <= self.canvas_height:
                self.draw_point(circumcenter.x, circumcenter.y, color="orange")
                tk_cx, tk_cy = self.logical_to_tk_coords(circumcenter.x, circumcenter.y)
                self.canvas.create_text(tk_cx + 5, tk_cy,
                                        text=f"({int(circumcenter.x)},{int(circumcenter.y)})",
                                        anchor="w", fill="purple", font=("Arial", 10, "bold"))
            else:
                print("注意：外心交點落在畫布範圍之外，點和座標將不會被繪製。")
            
            print("\n--- 計算並繪製最終 Voronoi 半線 (Rays) ---")
            move_distance = 5.0

            for edge in self.current_edges:
                origin_points = edge.origin_vertices
                other_point = [v for v in sorted_vertices if v not in origin_points][0]

                dx = edge.end.x - edge.start.x
                dy = edge.end.y - edge.start.y
                mag = math.sqrt(dx**2 + dy**2)
                if mag == 0: continue
                unit_vector_x, unit_vector_y = dx / mag, dy / mag

                p1_x = circumcenter.x + move_distance * unit_vector_x
                p1_y = circumcenter.y + move_distance * unit_vector_y
                
                p2_x = circumcenter.x - move_distance * unit_vector_x
                p2_y = circumcenter.y - move_distance * unit_vector_y

                dist1 = math.sqrt((p1_x - other_point.x)**2 + (p1_y - other_point.y)**2)
                dist2 = math.sqrt((p2_x - other_point.x)**2 + (p2_y - other_point.y)**2)

                if dist1 > dist2:
                    ray_direction_x, ray_direction_y = unit_vector_x, unit_vector_y
                    direction_desc = "正向"
                else:
                    ray_direction_x, ray_direction_y = -unit_vector_x, -unit_vector_y
                    direction_desc = "反向"

                ray_length = max(self.canvas_width, self.canvas_height) * 1.5 
                ray_end_x = circumcenter.x + ray_length * ray_direction_x
                ray_end_y = circumcenter.y + ray_length * ray_direction_y
                
                ray_end_vertex = Vertex(id=-1, x=ray_end_x, y=ray_end_y)
                
                voronoi_ray = Edge(id=edge.id, start_vertex=circumcenter, end_vertex=ray_end_vertex)

                print(f"中垂線(來自點 {origin_points[0].id},{origin_points[1].id}): dist1={dist1:.1f}, dist2={dist2:.1f} -> 保留 {direction_desc} 半線。")

                self.draw_edge_on_canvas(voronoi_ray, color="red", width=3)

            print("------------------------------------------------")

    def draw_point(self, x, y, color="black"):
        """在畫布上畫一個點，輸入為邏輯座標，內部轉換為 Tkinter 座標"""
        tk_x, tk_y = self.logical_to_tk_coords(x, y)
        r = 5 
        return self.canvas.create_oval(tk_x - r, tk_y - r, tk_x + r, tk_y + r, fill=color, outline=color)

    def on_canvas_click(self, event):
        """處理畫布點擊事件，將 Tkinter 座標轉換為邏輯座標顯示"""
        tk_x, tk_y = event.x, event.y
        logical_x, logical_y = self.tk_to_logical_coords(tk_x, tk_y)
        
        if self.clicked_point_id:
            self.canvas.delete(self.clicked_point_id)
        self.clicked_point_id = self.canvas.create_oval(tk_x - 4, tk_y - 4, tk_x + 4, tk_y + 4, outline="red", width=2)
        self.coord_label.config(text=f"座標: ({logical_x}, {logical_y})")

    def read_all_test_data(self):
        """讀取所有測資區塊。讀取的座標直接作為邏輯座標儲存。"""
        self.test_data_blocks = []
        try:
            with open(self.filename, 'r') as f:
                lines = f.readlines()
                i = 0
                while i < len(lines):
                    num_points_str = lines[i].strip()
                    if not num_points_str:
                        i += 1; continue
                    try:
                        num_points = int(num_points_str)
                    except ValueError:
                        i += 1; continue
                    if num_points == 0: break
                    current_block_points = []
                    i += 1
                    for _ in range(num_points):
                        if i < len(lines):
                            try:
                                x_str, y_str = lines[i].strip().split()
                                x, y = int(x_str), int(y_str)
                                current_block_points.append((x, y))
                                i += 1
                            except (ValueError, IndexError):
                                i += 1
                        else: break
                    self.test_data_blocks.append(current_block_points)
        except FileNotFoundError:
            print(f"錯誤: 檔案 '{self.filename}' 未找到。")
        except Exception as e:
            print(f"讀取檔案時發生錯誤: {e}")

    def load_next_test_data(self):
        """載入下一個測資區塊並繪製點。繪製時會轉換座標。"""
        if not self.test_data_blocks: return
        self.current_block_index = (self.current_block_index + 1) % len(self.test_data_blocks)
        self.canvas.delete("all")
        self.clicked_point_id = None
        points_to_draw = self.test_data_blocks[self.current_block_index]
        for x, y in points_to_draw:
            self.draw_point(x, y, color="blue")
            tk_x, tk_y = self.logical_to_tk_coords(x, y)
            self.canvas.create_text(tk_x + 5, tk_y, text=f"({x},{y})", anchor="w", fill="black", font=("Arial", 9))
        print(f"載入測資區塊 {self.current_block_index + 1}，包含 {len(points_to_draw)} 個點。")

# 主程式
if __name__ == "__main__":
    
    root = tk.Tk()
    # 請確保 "無註解Voronoi Diagra公開測資.txt" 檔案與此腳本在同一個目錄下
    app = PointDrawerApp(root, "無註解Voronoi Diagra公開測資.txt")
    root.mainloop()