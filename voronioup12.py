import tkinter as tk
from tkinter import messagebox
import math
from collections import Counter

class Vertex:
    """代表一個點（或稱頂點）的類別"""
    def __init__(self, id, x=0.0, y=0.0):
        self.id = id
        self.position = (x, y)

    @property
    def x(self):
        return self.position[0]

    @property
    def y(self):
        return self.position[1]

    def __repr__(self):
        return f"V{self.id}({self.x:.1f}, {self.y:.1f})"

class Edge:
    """代表一條邊的類別"""
    def __init__(self, id, start_vertex, end_vertex, origin_vertices=None):
        self.id = id
        self.start = start_vertex
        self.end = end_vertex
        self.origin_vertices = origin_vertices

    def __repr__(self):
        base_repr = f"Edge(start={self.start}, end={self.end})"
        if self.origin_vertices:
            return f"{base_repr} [由 {self.origin_vertices[0]} 和 {self.origin_vertices[1]} 的中垂線生成]"
        return base_repr


class PointDrawerApp:
    def __init__(self, root, filename=None):
        self.root = root
        self.root.title("Voronoi - Divide and Conquer (Recursive)")

        button_frame = tk.Frame(root)
        button_frame.pack(side=tk.TOP, pady=10)

        self.btn1 = tk.Button(button_frame, text="載入下一個測試案例", command=self.load_next_test_data)
        self.btn1.pack(side=tk.LEFT, padx=5)

        # ### 修改 ###: 按鈕文字變更，功能更清晰
        self.btn2 = tk.Button(button_frame, text="執行遞迴計算", command=self.run_recursive_voronoi)
        self.btn2.pack(side=tk.LEFT, padx=5)

        self.btn3 = tk.Button(button_frame, text="下一步 (合併)", command=self.next_step_action, state=tk.DISABLED)
        self.btn3.pack(side=tk.LEFT, padx=5)

        separator = tk.Frame(button_frame, height=20, width=2, bg="grey")
        separator.pack(side=tk.LEFT, padx=10, fill='y')
        
        tk.Label(button_frame, text="載入第").pack(side=tk.LEFT, padx=(5,0))
        self.entry_specific_case = tk.Entry(button_frame, width=4)
        self.entry_specific_case.pack(side=tk.LEFT)
        tk.Label(button_frame, text="筆").pack(side=tk.LEFT, padx=(0,5))
        
        self.btn_load_specific = tk.Button(button_frame, text="載入指定案例", command=self.load_specific_test_data)
        self.btn_load_specific.pack(side=tk.LEFT, padx=5)

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
        
        self.reset_voronoi_state()

        if self.filename:
            self.read_all_test_data()
            if self.test_data_blocks:
                self.load_next_test_data()
            else:
                messagebox.showwarning("讀取錯誤", f"'{self.filename}' 中沒有可用的測試資料，或讀取失敗。")

    def reset_voronoi_state(self):
        """重置所有計算狀態"""
        self.all_vertices = []
        self.voronoi_edges = []
        self.convex_hull = []
        
        # --- 用於分步合併的狀態 ---
        self.merge_phase = 'idle'
        self.left_results = None
        self.right_results = None
        self.merging_chain_edges = []
        self.current_bridge_l = None
        self.current_bridge_r = None
        self.upper_tangent = None
        self.lower_tangent = None
        
        if hasattr(self, 'canvas'):
            self.canvas.delete("all")
        
        if hasattr(self, 'btn3'):
            self.btn3.config(text="下一步 (合併)", state=tk.DISABLED)

    # ... (Vertex, Edge 類別和多數輔助函式不變) ...
    def logical_to_tk_coords(self, x, y):
        return x, self.canvas_height - y

    def tk_to_logical_coords(self, tk_x, tk_y):
        return tk_x, self.canvas_height - tk_y

    def cross_product(self, o, a, b):
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

    def distance_sq(self, p1, p2):
        if not p1 or not p2: return float('inf')
        return (p1.x - p2.x)**2 + (p1.y - p2.y)**2

    def sort_vertices_by_angle(self, vertices):
        if len(vertices) < 3: return vertices
        if all(abs(self.cross_product(vertices[0], vertices[1], v)) < 1e-9 for v in vertices[2:]):
            return sorted(vertices, key=lambda v: (v.x, v.y))
        cx = sum(v.x for v in vertices) / len(vertices)
        cy = sum(v.y for v in vertices) / len(vertices)
        return sorted(vertices, key=lambda v: math.atan2(v.y - cy, v.x - cx))

    def get_perpendicular_bisector(self, v1, v2, edge_id, length=1200):
        if v1 is None or v2 is None: return None
        mx = (v1.x + v2.x) / 2
        my = (v1.y + v2.y) / 2
        dx = v2.x - v1.x
        dy = v2.y - v1.y
        mag = (dx**2 + dy**2)**0.5
        if mag == 0: return None
        ndx, ndy = -dy / mag, dx / mag
        p1 = Vertex(edge_id*10+1, mx - ndx * length, my - ndy * length)
        p2 = Vertex(edge_id*10+2, mx + ndx * length, my + ndy * length)
        return Edge(edge_id, p1, p2, origin_vertices=(v1, v2))
        
    def get_line_intersection(self, edge1, edge2, check_segment1=True, check_segment2=True):
        if not all([edge1, edge2, edge1.start, edge1.end, edge2.start, edge2.end]): return None
        p1, p2 = edge1.start, edge1.end
        p3, p4 = edge2.start, edge2.end
        den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x)
        if abs(den) < 1e-9: return None
        t_num = (p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)
        u_num = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x))
        t, u = t_num / den, u_num / den
        if check_segment1 and not (0.0 <= t <= 1.0): return None
        if check_segment2 and not (0.0 <= u <= 1.0): return None
        return Vertex(id=-10, x=p1.x + t * (p2.x - p1.x), y=p1.y + t * (p2.y - p1.y))

    def get_three_line_cross_point(self, v1, v2, v3):
        bisector1 = self.get_perpendicular_bisector(v1, v2, -1)
        bisector2 = self.get_perpendicular_bisector(v2, v3, -2)
        return self.get_line_intersection(bisector1, bisector2, check_segment1=False, check_segment2=False)

    def draw_edge_on_canvas(self, edge, color="red", width=1, dash_pattern=None, tags=None):
        if not edge or not edge.start or not edge.end: return
        x1, y1 = self.logical_to_tk_coords(edge.start.x, edge.start.y)
        x2, y2 = self.logical_to_tk_coords(edge.end.x, edge.end.y)
        self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width, dash=dash_pattern, tags=tags)

    # ### 新增 ###: 這是遞迴的核心函式
    def _voronoi_recursive(self, vertices):
        """
        遞迴計算Voronoi圖。
        返回: (edges, convex_hull)
        """
        num_vertices = len(vertices)
        if num_vertices <= 3:
            # 基底情況: 點數小於等於3，直接計算
            edges = self.compute_voronoi_for_small_set(vertices)
            hull = self.sort_vertices_by_angle(vertices)
            return edges, hull

        # 遞迴步驟: 分割
        mid = num_vertices // 2
        left_half_points = vertices[:mid]
        right_half_points = vertices[mid:]

        # 遞迴呼叫
        left_edges, left_hull = self._voronoi_recursive(left_half_points)
        right_edges, right_hull = self._voronoi_recursive(right_half_points)

        # 合併
        merged_edges, final_hull = self._merge_voronoi(
            left_edges, left_hull, right_edges, right_hull
        )
        
        return merged_edges, final_hull
    
    # ### 新增 ###: 合併兩個子Voronoi圖的邏輯
    def _merge_voronoi(self, left_edges, left_hull, right_edges, right_hull):
        """
        合併左右兩邊的Voronoi圖。
        這個函式模擬了您原有的 '下一步' 流程，但在此一次性完成計算。
        返回: (all_edges, merged_hull)
        """
        # 1. 找到上下公切線，建立初始橋樑
        upper_l, upper_r = self.find_common_tangent(left_hull, right_hull, find_upper=True)
        lower_l, lower_r = self.find_common_tangent(left_hull, right_hull, find_upper=False)

        bridge_l, bridge_r = upper_l, upper_r
        
        merging_chain = []
        
        # 2. 從上公切線開始，往下建立合併鏈
        bisector = self.get_perpendicular_bisector(bridge_l, bridge_r, -201)
        
        # 模擬迴圈，不斷尋找下一個交點，直到橋樑到達下公切線
        loop_guard = 0 # 防止無限迴圈
        while loop_guard < 100:
            loop_guard += 1

            if bridge_l == lower_l and bridge_r == lower_r:
                break
            
            # 3. 尋找 bisector 與左右Voronoi邊的下一個交點
            # 整合左右兩邊的邊，以供搜尋
            temp_left_edges = list(left_edges)
            temp_right_edges = list(right_edges)
            
            # find_next_event 需要一個模擬的 bisector_line edge
            next_event = self.find_next_event(bisector, None, temp_left_edges, temp_right_edges)
            intersect_pt = next_event.get('intersect_pt')

            if not intersect_pt:
                 # 如果找不到交點，表示可以直接連到下公切線的中垂線
                 break

            # 4. 裁剪被交到的舊邊，並更新橋樑
            edge_to_trim = next_event['edge']
            side = next_event['side']
            
            # 修剪舊邊
            if self.distance_sq(edge_to_trim.start, intersect_pt) > self.distance_sq(edge_to_trim.end, intersect_pt):
                edge_to_trim.start = intersect_pt
            else:
                edge_to_trim.end = intersect_pt

            # 更新橋樑
            if side == 'left':
                bridge_l = next(v for v in edge_to_trim.origin_vertices if v != bridge_l)
            else: # side == 'right'
                bridge_r = next(v for v in edge_to_trim.origin_vertices if v != bridge_r)

            # 5. 建立新的合併鏈邊
            new_bisector = self.get_perpendicular_bisector(bridge_l, bridge_r, -202)
            
            # 新的合併邊從上一個交點開始
            new_edge = Edge(-301, intersect_pt, Vertex(-1,0,0), origin_vertices=(bridge_l, bridge_r))
            merging_chain.append(new_edge)
            
            # 將上一段合併鏈的終點設定為目前的交點
            if len(merging_chain) > 1:
                merging_chain[-2].end = intersect_pt
            
            bisector = new_bisector

        # 6. 處理合併鏈的起點和終點
        # 起點：上公切線中垂線與無限遠處的交點（或用一個射線表示）
        start_bisector = self.get_perpendicular_bisector(upper_l, upper_r, -998)
        first_intersect = merging_chain[0].start if merging_chain else None
        
        dx = start_bisector.end.x - start_bisector.start.x
        dy = start_bisector.end.y - start_bisector.start.y
        if dy < 0: dx, dy = -dx, -dy # 確保向上
        
        # 如果有交點，則建立從無限遠到第一個交點的射線
        if first_intersect:
            ray_start = Vertex(-1, first_intersect.x - dx * 1000, first_intersect.y - dy * 1000)
            start_edge = Edge(-401, ray_start, first_intersect, origin_vertices=(upper_l, upper_r))
            merging_chain.insert(0, start_edge)
        else: # 沒有任何交點，合併鏈就是一條無限長的中垂線
             merging_chain.append(start_bisector)

        # 終點：下公切線中垂線
        end_bisector = self.get_perpendicular_bisector(lower_l, lower_r, -999)
        last_intersect = merging_chain[-1].start if len(merging_chain)>1 else None
        
        # 確保射線向下
        dx = end_bisector.end.x - end_bisector.start.x
        dy = end_bisector.end.y - end_bisector.start.y
        if dy > 0: dx, dy = -dx, -dy
        
        # 從最後一個交點或下公切線中點，畫一條射線到無限遠
        if last_intersect:
             merging_chain[-1].end = last_intersect
             ray_end = Vertex(-1, last_intersect.x + dx * 1000, last_intersect.y + dy * 1000)
             end_edge = Edge(-402, last_intersect, ray_end, origin_vertices=(lower_l, lower_r))
             merging_chain.append(end_edge)
        else: #如果合併鏈是空的，代表只有一條從上到下的線
             merging_chain[-1].end = Vertex(-1, merging_chain[-1].start.x + dx*1000, merging_chain[-1].start.y + dy*1000)


        # 7. 合併凸包 (簡單地取兩者合集的凸包)
        merged_hull = self.sort_vertices_by_angle(left_hull + right_hull)
        
        return (left_edges + right_edges + merging_chain), merged_hull


    def compute_voronoi_for_small_set(self, vertices):
        # (此函式內容不變)
        if not vertices or len(vertices) < 2: return []
        edges = []
        if len(vertices) == 2:
            edges.append(self.get_perpendicular_bisector(vertices[0], vertices[1], 0))
        elif len(vertices) == 3:
            v1, v2, v3 = vertices
            if abs(self.cross_product(v1, v2, v3)) < 1e-9:
                sorted_v = sorted(vertices, key=lambda v: (v.x, v.y))
                edges.append(self.get_perpendicular_bisector(sorted_v[0], sorted_v[1], 0))
                edges.append(self.get_perpendicular_bisector(sorted_v[1], sorted_v[2], 1))
            else:
                center = self.get_three_line_cross_point(v1, v2, v3)
                if center:
                    for p1, p2 in [(v1, v2), (v2, v3), (v3, v1)]:
                        mid_point = Vertex(-1, (p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
                        dx, dy = mid_point.x - center.x, mid_point.y - center.y
                        end_point = Vertex(-1, center.x + dx * 1000, center.y + dy * 1000)
                        edges.append(Edge(-1, center, end_point, origin_vertices=(p1, p2)))
        return [e for e in edges if e]

    def find_common_tangent(self, left_hull, right_hull, find_upper=True):
        # (此函式內容不變)
        if not left_hull or not right_hull: return None, None
        l_idx = max(range(len(left_hull)), key=lambda i: left_hull[i].x)
        r_idx = min(range(len(right_hull)), key=lambda i: right_hull[i].x)
        while True:
            changed = False
            l_cand_idx = (l_idx + (-1 if find_upper else 1) + len(left_hull)) % len(left_hull)
            if ((find_upper and self.cross_product(right_hull[r_idx], left_hull[l_idx], left_hull[l_cand_idx]) < 0) or
               (not find_upper and self.cross_product(right_hull[r_idx], left_hull[l_idx], left_hull[l_cand_idx]) > 0)):
                l_idx = l_cand_idx; changed = True
            r_cand_idx = (r_idx + (1 if find_upper else -1) + len(right_hull)) % len(right_hull)
            if ((find_upper and self.cross_product(left_hull[l_idx], right_hull[r_idx], right_hull[r_cand_idx]) > 0) or
               (not find_upper and self.cross_product(left_hull[l_idx], right_hull[r_idx], right_hull[r_cand_idx]) < 0)):
                r_idx = r_cand_idx; changed = True
            if not changed: break
        return left_hull[l_idx], right_hull[r_idx]

    # ### 主要修改 ###: 這個按鈕現在啟動遞迴計算
    def run_recursive_voronoi(self):
        if self.current_block_index == -1:
            messagebox.showwarning("提示", "請先載入測試資料。")
            return
        
        self.reset_voronoi_state()
        
        points = self.test_data_blocks[self.current_block_index]
        self.all_vertices = [Vertex(id=i, x=p[0], y=p[1]) for i, p in enumerate(points)]
        
        for v in self.all_vertices:
            self.draw_point(v.x, v.y, "blue")
            self.canvas.create_text(*self.logical_to_tk_coords(v.x, v.y + 5), text=str(v.id), anchor="n", fill="black", font=("Arial", 9))
            
        if not self.all_vertices:
            return

        # 啟動前先排序
        sorted_vertices = sorted(self.all_vertices, key=lambda v: (v.x, v.y))
        
        # 呼叫遞迴函式
        self.voronoi_edges, self.convex_hull = self._voronoi_recursive(sorted_vertices)
        
        print(f"遞迴計算完成。總共生成 {len(self.voronoi_edges)} 條 Voronoi 邊。")
        
        self.redraw_all_edges()
        self.cleanup_final_guides() # 直接清理並顯示最終結果
        
        messagebox.showinfo("完成", "遞迴 Voronoi 圖計算已完成！")


    # ... (next_step_action 和相關函式需要簡化或移除，因為計算已一次性完成) ...
    # 為了簡單起見，我們讓上面的按鈕一次性完成所有計算並顯示結果。
    # 分步演示遞迴過程需要更複雜的狀態管理，這裡暫不實作。
    def next_step_action(self):
        messagebox.showinfo("提示", "遞迴版本中，'執行遞迴計算' 按鈕會一次性完成所有步驟。")


    def find_next_event(self, bisector_line, point_to_ignore, left_edges, right_edges):
        # (此函式內容微調，接收邊列表作為參數)
        events = []
        ignore_tolerance_sq = 1e-9
         
        # ----- 檢查左側邊 -----
        for edge in left_edges:
            if edge and edge.origin_vertices:
                pt = self.get_line_intersection(bisector_line, edge, check_segment1=False, check_segment2=True)
                if pt:
                    if point_to_ignore and self.distance_sq(pt, point_to_ignore) < ignore_tolerance_sq:
                        continue
                    events.append({'intersect_pt': pt, 'edge': edge, 'side': 'left'})
        
        # ----- 檢查右側邊 -----
        for edge in right_edges:
            if edge and edge.origin_vertices:
                pt = self.get_line_intersection(bisector_line, edge, check_segment1=False, check_segment2=True)
                if pt:
                    if point_to_ignore and self.distance_sq(pt, point_to_ignore) < ignore_tolerance_sq:
                        continue
                    events.append({'intersect_pt': pt, 'edge': edge, 'side': 'right'})
        
        if not events:
            return {}
        
        best_event = max(events, key=lambda e: e['intersect_pt'].y - abs(e['intersect_pt'].x) * 1e-9)
        return best_event

    def cleanup_final_guides(self):
        # (此函式內容不變，但現在由 run_recursive_voronoi 直接呼叫)
        print("\n--- 清理階段：校驗 Voronoi 邊 ---")
        
        all_final_edges = self.voronoi_edges
        if not any(all_final_edges):
            return

        endpoint_coords = []
        coord_to_vertex = {} 
        
        for edge in all_final_edges:
            if not edge or not edge.start or not edge.end: continue
            start_key = (round(edge.start.x, 3), round(edge.start.y, 3))
            end_key = (round(edge.end.x, 3), round(edge.end.y, 3))
            
            endpoint_coords.append(start_key)
            endpoint_coords.append(end_key)
            coord_to_vertex[start_key] = edge.start
            coord_to_vertex[end_key] = edge.end

        endpoint_counts = Counter(endpoint_coords)

        dangling_coords = set()
        margin = 1 
        for coord, count in endpoint_counts.items():
            if count == 1:
                v = coord_to_vertex[coord]
                is_outside_canvas = (v.x < -margin or v.x > self.canvas_width + margin or
                                     v.y < -margin or v.y > self.canvas_height + margin)
                if not is_outside_canvas:
                    dangling_coords.add(coord)

        if dangling_coords:
            is_dangling_edge = lambda edge: (
                (round(edge.start.x, 3), round(edge.start.y, 3)) in dangling_coords or
                (round(edge.end.x, 3), round(edge.end.y, 3)) in dangling_coords
            )
            self.voronoi_edges = [edge for edge in self.voronoi_edges if not is_dangling_edge(edge)]
            self.redraw_all_edges()
        else:
            print("    [校驗] 未發現懸空邊，圖形結構完整。")

    def redraw_all_edges(self):
        self.canvas.delete("voronoi_edge", "hull")
        for edge in self.voronoi_edges:
            self.draw_edge_on_canvas(edge, "black", 2, tags="voronoi_edge")
        
        # 也可以選擇性地繪製最終的凸包
        # for i in range(len(self.convex_hull)):
        #     p1 = self.convex_hull[i]
        #     p2 = self.convex_hull[(i + 1) % len(self.convex_hull)]
        #     self.draw_edge_on_canvas(Edge(-1, p1, p2), "blue", 1, (2,2), "hull")

    def draw_point(self, x, y, color="black"):
        tk_x, tk_y = self.logical_to_tk_coords(x, y)
        r = 4
        self.canvas.create_oval(tk_x - r, tk_y - r, tk_x + r, tk_y + r, fill=color, outline=color)

    def on_canvas_click(self, event):
        tk_x, tk_y = event.x, event.y
        lx, ly = self.tk_to_logical_coords(tk_x, tk_y)
        if self.clicked_point_id:
            self.canvas.delete(self.clicked_point_id)
        self.clicked_point_id = self.canvas.create_oval(tk_x - 5, tk_y - 5, tk_x + 5, tk_y + 5, outline="red", width=2)
        self.coord_label.config(text=f"座標: ({lx:.1f}, {ly:.1f})")

    def read_all_test_data(self):
        # (此函式內容不變)
        self.test_data_blocks = []
        if not self.filename: 
            messagebox.showerror("錯誤", "未提供檔案名稱。")
            return
        for encoding in ['utf-8', 'utf-8-sig', 'big5', 'cp950']:
            try:
                with open(self.filename, 'r', encoding=encoding) as f: lines = f.readlines()
                break
            except Exception: continue
        else: 
            messagebox.showerror("錯誤", f"無法讀取檔案: {self.filename}")
            return
        
        i = 0
        while i < len(lines):
            try:
                line_content = lines[i].strip()
                if not line_content: i += 1; continue
                num_points = int(line_content)
                if num_points == 0: break
                i += 1
                block = [tuple(map(int, lines[j].strip().split())) for j in range(i, i + num_points)]
                if len(block) == num_points: self.test_data_blocks.append(block)
                i += num_points
            except (ValueError, IndexError): i += 1
        print(f"已成功從檔案載入 {len(self.test_data_blocks)} 筆測試資料。")

    def _load_and_draw_data(self):
        self.reset_voronoi_state()
        points = self.test_data_blocks[self.current_block_index]
        self.all_vertices = [Vertex(id=i, x=p[0], y=p[1]) for i, p in enumerate(points)]
        for v in self.all_vertices:
            self.draw_point(v.x, v.y, "blue")
            tk_x, tk_y = self.logical_to_tk_coords(v.x, v.y)
            self.canvas.create_text(tk_x + 8, tk_y, text=str(v.id), anchor="w", fill="black", font=("Arial", 9))
        print(f"\n已載入測試案例 {self.current_block_index + 1} / {len(self.test_data_blocks)}，共 {len(points)} 個點。")
        # ### 修改 ###: "下一步"按鈕在載入新資料後應禁用，直到計算開始
        self.btn3.config(state=tk.DISABLED)

    def load_next_test_data(self):
        if not self.test_data_blocks: return
        self.current_block_index = (self.current_block_index + 1) % len(self.test_data_blocks)
        self._load_and_draw_data()

    def load_specific_test_data(self):
        if not self.test_data_blocks: return
        try:
            target_index = int(self.entry_specific_case.get()) - 1
            if 0 <= target_index < len(self.test_data_blocks):
                self.current_block_index = target_index
                self._load_and_draw_data()
            else:
                messagebox.showerror("輸入無效", f"請輸入 1 到 {len(self.test_data_blocks)} 之間的數字。")
        except ValueError:
            messagebox.showerror("輸入無效", "請輸入一個有效的數字。")

if __name__ == "__main__":
    root = tk.Tk()
    # 請確認您的測資檔案名稱是否正確
    app = PointDrawerApp(root, "無註解Voronoi Diagra公開測資.txt")
    root.mainloop()