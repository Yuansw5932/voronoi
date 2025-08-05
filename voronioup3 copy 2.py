import tkinter as tk
from tkinter import messagebox
import math

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
        # 為了偵錯，顯示更詳細的座標
        return f"V{self.id}({self.x:.1f}, {self.y:.1f})"

class Edge:
    """代表一條邊的類別"""
    def __init__(self, id, start_vertex, end_vertex, origin_vertices=None):
        self.id = id
        self.start = start_vertex
        self.end = end_vertex
        self.origin_vertices = origin_vertices

    def __repr__(self):
        # 為了偵錯，顯示更詳細的邊資訊
        base_repr = f"Edge(start={self.start}, end={self.end})"
        # 如果這條邊是由兩個原始頂點生成的中垂線，則一併顯示
        if self.origin_vertices:
            return f"{base_repr} [由 {self.origin_vertices[0]} 和 {self.origin_vertices[1]} 的中垂線生成]"
        return base_repr


class PointDrawerApp:
    def __init__(self, root, filename=None):
        self.root = root
        self.root.title("Voronoi divide and conquer")

        button_frame = tk.Frame(root)
        button_frame.pack(side=tk.TOP, pady=10)

        self.btn1 = tk.Button(button_frame, text="載入下一個測試案例", command=self.load_next_test_data)
        self.btn1.pack(side=tk.LEFT, padx=5)

        self.btn2 = tk.Button(button_frame, text="分割與排序", command=self.process_and_draw_points)
        self.btn2.pack(side=tk.LEFT, padx=5)

        self.btn3 = tk.Button(button_frame, text="下一步", command=self.next_step_action)
        self.btn3.pack(side=tk.LEFT, padx=5)

        # <--- 新增開始: 指定載入功能的UI元件 ---
        separator = tk.Frame(button_frame, height=20, width=2, bg="grey")
        separator.pack(side=tk.LEFT, padx=10, fill='y')
        
        tk.Label(button_frame, text="載入第").pack(side=tk.LEFT, padx=(5,0))
        self.entry_specific_case = tk.Entry(button_frame, width=4)
        self.entry_specific_case.pack(side=tk.LEFT)
        tk.Label(button_frame, text="筆").pack(side=tk.LEFT, padx=(0,5))
        
        self.btn_load_specific = tk.Button(button_frame, text="載入指定案例", command=self.load_specific_test_data)
        self.btn_load_specific.pack(side=tk.LEFT, padx=5)
        # <--- 新增結束 ---

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
        self.left_half_points = []
        self.right_half_points = []
        self.left_hull = None
        self.right_hull = None
        self.left_voronoi_edges = []
        self.right_voronoi_edges = []
        
        self.merging_chain_edges = []
        
        self.merge_phase = 'idle'
        
        self.current_bridge_l = None
        self.current_bridge_r = None
        self.upper_tangent = None
        self.lower_tangent = None
        
        if hasattr(self, 'canvas'):
            print("\n日誌: 呼叫 reset_voronoi_state -> 移除所有畫布物件 (all)，原因：重置整個場景以載入新案例或重新開始。")
            self.canvas.delete("all")
        
        if hasattr(self, 'btn3'):
            self.btn3.config(text="下一步", state=tk.NORMAL)


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

    # ----- 函式修正開始 -----
    def get_line_intersection(self, edge1, edge2, check_segment1=True, check_segment2=True):
        """
        計算兩條線(或線段)的交點。
        :param edge1: 第一條邊
        :param edge2: 第二條邊
        :param check_segment1: 是否檢查交點是否在 edge1 線段上
        :param check_segment2: 是否檢查交點是否在 edge2 線段上
        :return: Vertex 物件或 None
        """
        if not all([edge1, edge2, edge1.start, edge1.end, edge2.start, edge2.end]):
            return None
            
        p1, p2 = edge1.start, edge1.end
        p3, p4 = edge2.start, edge2.end

        den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x)
        if abs(den) < 1e-9:
            return None  # 線平行或共線

        t_num = (p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)
        u_num = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x))

        t = t_num / den
        u = u_num / den

        # 根據參數檢查交點是否在線段範圍內
        # t 代表交點在 edge1 上的比例，u 代表在 edge2 上的比例
        if check_segment1 and not (0.0 <= t <= 1.0):
            return None
        if check_segment2 and not (0.0 <= u <= 1.0):
            return None

        # 計算交點座標
        x = p1.x + t * (p2.x - p1.x)
        y = p1.y + t * (p2.y - p1.y)
        
        return Vertex(id=-10, x=x, y=y)
    # ----- 函式修正結束 -----

    def get_three_line_cross_point(self, v1, v2, v3):
        # ----- 呼叫點修正 -----
        # 尋找外心時，中垂線應視為無限長，故不檢查線段邊界
        bisector1 = self.get_perpendicular_bisector(v1, v2, -1)
        bisector2 = self.get_perpendicular_bisector(v2, v3, -2)
        return self.get_line_intersection(bisector1, bisector2, check_segment1=False, check_segment2=False)

    def draw_edge_on_canvas(self, edge, color="red", width=1, dash_pattern=None, tags=None):
        if not edge or not edge.start or not edge.end: return
        x1, y1 = self.logical_to_tk_coords(edge.start.x, edge.start.y)
        x2, y2 = self.logical_to_tk_coords(edge.end.x, edge.end.y)
        self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width, dash=dash_pattern, tags=tags)

    def compute_voronoi_for_small_set(self, vertices):
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

    def process_and_draw_points(self):
        if self.current_block_index == -1: print("請先載入測試資料。"); return
        self.reset_voronoi_state() # 這裡會重置按鈕
        points = self.test_data_blocks[self.current_block_index]
        vertices = [Vertex(id=i, x=p[0], y=p[1]) for i, p in enumerate(points)]
        for v in vertices: self.draw_point(v.x, v.y, "blue"); self.canvas.create_text(*self.logical_to_tk_coords(v.x, v.y), text=str(v.id), anchor="n", fill="black", font=("Arial", 9))
        if len(vertices) <= 3: self.left_voronoi_edges = self.compute_voronoi_for_small_set(vertices); self.redraw_all_edges(); return
        
        x_sorted = sorted(vertices, key=lambda v: (v.x, v.y)); mid = len(x_sorted) // 2
        self.left_half_points, self.right_half_points = x_sorted[:mid], x_sorted[mid:]
        divider_x = (self.left_half_points[-1].x + self.right_half_points[0].x) / 2.0
        self.draw_edge_on_canvas(Edge(-100, Vertex(-98, divider_x, 0), Vertex(-99, divider_x, 600)), "blue", 2, (5, 3), "divider")
        self.left_hull = self.sort_vertices_by_angle(self.left_half_points); self.right_hull = self.sort_vertices_by_angle(self.right_half_points)
        for i, v in enumerate(self.left_hull): self.canvas.create_text(*self.logical_to_tk_coords(v.x, v.y+5), text=f"L{i}", fill="green", font=("Arial", 10, "bold"))
        for i, v in enumerate(self.right_hull): self.canvas.create_text(*self.logical_to_tk_coords(v.x, v.y+5), text=f"R{i}", fill="purple", font=("Arial", 10, "bold"))
        self.merge_phase = 'ready_to_merge'; print("已完成分割與排序，準備合併。")

    def next_step_action(self):
        actions = {
            'idle': lambda: print("請先點擊「分割與排序」。"),
            'ready_to_merge': self.draw_initial_bridge,
            'bridge_drawn': self.start_merging_chain,
            'merging': self.merge_voronoi_step,
            'complete': self.cleanup_final_guides,
            'cleaned_up': lambda: print("已清理輔助線。請載入下一個測試案例。")
        }
        actions.get(self.merge_phase, lambda: None)()

    def draw_initial_bridge(self):
        print("\n--- 第 1 步：計算子問題並繪製初始橋樑 ---")
        self.canvas.delete("voronoi_edge", "bridge_line")
        
        self.left_voronoi_edges = self.compute_voronoi_for_small_set(self.left_half_points)
        self.right_voronoi_edges = self.compute_voronoi_for_small_set(self.right_half_points)
        
        self.upper_tangent = self.find_common_tangent(self.left_hull, self.right_hull, True)
        self.lower_tangent = self.find_common_tangent(self.left_hull, self.right_hull, False)
        if not self.upper_tangent[0] or not self.upper_tangent[1]:
            self.merge_phase = 'complete'; return
            
        self.current_bridge_l, self.current_bridge_r = self.upper_tangent
        print(f"上切線 (初始橋樑): {self.current_bridge_l} -> {self.current_bridge_r}")
        self.draw_edge_on_canvas(Edge(-500, *self.current_bridge), "black", 1, (2,2), "bridge_line")
        
        self.merge_phase = 'bridge_drawn'
        self.redraw_all_edges()


    def start_merging_chain(self):
        print("\n--- 第 2 步：繪製第一條合併線 ---")
        initial_bisector = self.get_perpendicular_bisector(*self.current_bridge, -201)
        first_event = self.find_next_event(initial_bisector, point_to_ignore=None)
        first_intersect = first_event.get('intersect_pt')
        
        if not first_intersect:
            self.merge_phase = 'complete'
            self.btn3.config(text="清理輔助線")
            self.redraw_all_edges()
            return

        dx = initial_bisector.end.x - initial_bisector.start.x
        dy = initial_bisector.end.y - initial_bisector.start.y
        if dy < 0: dx, dy = -dx, -dy
        ray_end = Vertex(-1, first_intersect.x + dx * 1000, first_intersect.y + dy * 1000)
        edge = Edge(-301, first_intersect, ray_end, origin_vertices=self.current_bridge)

        self.merging_chain_edges.append(edge)
        self.update_bridge_and_trim(first_event)
        self.merge_phase = 'merging'

        self.draw_edge_on_canvas(edge, "black", 3, None, "hi")

        self.redraw_all_edges()


    def merge_voronoi_step(self):
        if self.merge_phase != 'merging' or not self.merging_chain_edges: return
        
        print(f"\n--- 第 {len(self.merging_chain_edges) + 2} 步：尋找下一個交點 ---")

        if self.current_bridge_l == self.lower_tangent[0] and self.current_bridge_r == self.lower_tangent[1]: #bug for last line cant be draw
             print("橋樑已抵達下公切線，合併完成！")
             for i, edge in enumerate(self.merging_chain_edges):
                is_predictive = (i == len(self.merging_chain_edges) - 1)
                
                color = "red" if is_predictive else "black"
                dash = (5,3) if is_predictive and self.merge_phase != 'complete' else None
                width = 2
                self.draw_edge_on_canvas(edge, color, width, dash, "hi")
                

             last_ray = self.merging_chain_edges[-1]
             p_bisector = self.get_perpendicular_bisector(*self.lower_tangent, -999)
             # ----- 呼叫點修正 -----
             # last_ray 是一條從上一交點出發的射線(線段)，而 p_bisector 是無限長的中垂線
             # 所以 edge1 (last_ray) 要檢查，edge2 (p_bisector) 不用檢查
             final_intersection = self.get_line_intersection(last_ray, p_bisector, check_segment1=True, check_segment2=False)
             if final_intersection: last_ray.end = final_intersection
             self.merge_phase = 'complete'
             self.btn3.config(text="清理輔助線")
             self.redraw_all_edges()
             return

        last_known_intersection = self.merging_chain_edges[-1].start
        current_bisector = self.get_perpendicular_bisector(*self.current_bridge, -202)
        next_event = self.find_next_event(current_bisector, point_to_ignore=last_known_intersection)
        next_intersect = next_event.get('intersect_pt')
        
        if not next_intersect:
            print("找不到新的交點，合併結束。")
            self.merge_phase = 'complete'
            self.btn3.config(text="清理輔助線")
            self.redraw_all_edges()
            return
        
        self.merging_chain_edges[-1].end = next_intersect

        self.update_bridge_and_trim(next_event)

        new_bisector = self.get_perpendicular_bisector(*self.current_bridge, -203)
        dx, dy = new_bisector.end.x - new_bisector.start.x, new_bisector.end.y - new_bisector.start.y
        if dy > 0: dx, dy = -dx, -dy # 確保射線方向朝下
        ray_end = Vertex(-1, next_intersect.x + dx * 1000, next_intersect.y + dy * 1000)
        self.merging_chain_edges.append(Edge(-204, next_intersect, ray_end, origin_vertices=self.current_bridge))

        self.redraw_all_edges()

    def find_next_event(self, bisector_line, point_to_ignore=None):
        events = []
        ignore_tolerance_sq = 1e-9

        #self.draw_edge_on_canvas(bisector_line, "red", 3, None, "e")
         
        print(f"    [find_next_event] 正在以 {bisector_line} 尋找新事件...")
        if point_to_ignore:
            print(f"    [find_next_event] (將忽略此點附近的交點: {point_to_ignore})")

        # ----- 檢查左側邊 -----
        for edge in self.left_voronoi_edges:
            if edge and edge.origin_vertices :
                # 【新增日誌】記錄哪些邊通過了篩選，正準備被用來計算交點
                print(f"    [find_next_event] -> 正在檢查 (左側/綠邊): {edge}")
                
                pt = self.get_line_intersection(bisector_line, edge, check_segment1=False, check_segment2=True)
                if pt:
                    if point_to_ignore and self.distance_sq(pt, point_to_ignore) < ignore_tolerance_sq:
                        continue
                    print(f"        -> 候選(左): 交點 {pt} (來自綠邊 {edge})")
                    events.append({'intersect_pt': pt, 'edge': edge, 'side': 'left'})
        
        # ----- 檢查右側邊 -----
        for edge in self.right_voronoi_edges:
            if edge and edge.origin_vertices :
                # 【新增日誌】記錄哪些邊通過了篩選，正準備被用來計算交點
                print(f"    [find_next_event] -> 正在檢查 (右側/紫邊): {edge}")

                pt = self.get_line_intersection(bisector_line, edge, check_segment1=False, check_segment2=True)
                if pt:
                    if point_to_ignore and self.distance_sq(pt, point_to_ignore) < ignore_tolerance_sq:
                        continue
                    print(f"        -> 候選(右): 交點 {pt} (來自紫邊 {edge})")
                    events.append({'intersect_pt': pt, 'edge': edge, 'side': 'right'})
        
        if not events:
            print("    [find_next_event] 未找到任何有效交點。")
            return {}
        
        # 尋找Y座標最大（最上方）的交點作為下一個事件點
        best_event = max(events, key=lambda e: e['intersect_pt'].y - abs(e['intersect_pt'].x) * 1e-9)
        print(f"    [find_next_event] 從 {len(events)} 個候選中選擇最高點: {best_event['intersect_pt']} (來自 {best_event['side']} 側)")
        return best_event


    def update_bridge_and_trim(self, event):
        """
        更穩健地修剪 Voronoi 邊緣。
        明確檢查兩個端點，只修剪位於新邊界「無效側」的端點。
        """
        edge, side, pt = event.get('edge'), event.get('side'), event.get('intersect_pt')
        if not all([edge, side, pt]): return

        original_start, original_end = edge.start, edge.end

        # 根據邊緣屬於左半邊還是右半邊，定義什麼是「無效區域」
        # - 對於右邊的邊(purple)，如果其端點離「左邊」的橋點更近，則該端點無效。
        # - 對於左邊的邊(green)，如果其端點離「右邊」的橋點更近，則該端點無效。
        if side == 'right':
            is_invalid = lambda p: self.distance_sq(p, self.current_bridge_l) < self.distance_sq(p, self.current_bridge_r)
        else:  # side == 'left'
            is_invalid = lambda p: self.distance_sq(p, self.current_bridge_r) < self.distance_sq(p, self.current_bridge_l)

        # 明確檢查兩個端點
        start_is_invalid = is_invalid(edge.start)
        end_is_invalid = is_invalid(edge.end)

        # 根據檢查結果進行修剪
        if start_is_invalid and not end_is_invalid:
            # Start 端點無效，將其位置更新為交叉點 pt
            edge.start = pt
        elif end_is_invalid and not start_is_invalid:
            # End 端點無效，將其位置更新為交叉點 pt
            edge.end = pt
        elif start_is_invalid and end_is_invalid:
            # 理論上，如果交叉點有效，只會有一個端點無效。
            # 如果兩者都無效，代表整條邊都可能被吞噬，這在某些情況下可能發生。
            # 為求穩定，這裡先印出警告，並採用舊的邏輯做降級處理。
            print(f"警告: 邊 {edge} 的兩端點都無效，可能需要更複雜的處理。")
            if self.distance_sq(edge.start, pt) > self.distance_sq(edge.end, pt):
                edge.start = pt
            else:
                edge.end = pt
        # else: 兩個端點都有效，這意味著交叉點可能在線段外部，
        # find_next_event 應該已經過濾這種情況，理論上不會進入此分支。

        edge_color = "綠色 (左)" if side == 'left' else "紫色 (右)"
        original_edge_repr = f"Edge(start={original_start}, end={original_end})"
        print("\n" + "="*20 + " Voronoi 邊緣變更 " + "="*20)
        print(f"【追蹤】: 一條 {edge_color} 邊被交點 {pt} 裁剪。")
        print(f"    - 原始邊: {original_edge_repr}")
        print(f"    - 更新後邊: {edge}")
        print("="*58 + "\n")

        self.canvas.delete("bridge_line")
        if side == 'left':
            new_l = next(v for v in edge.origin_vertices if v != self.current_bridge_l)
            print(f"事件: 橋樑左側從 {self.current_bridge_l} 移動至 {new_l}。")
            self.current_bridge_l = new_l
        else:
            new_r = next(v for v in edge.origin_vertices if v != self.current_bridge_r)
            print(f"事件: 橋樑右側從 {self.current_bridge_r} 移動至 {new_r}。")
            self.current_bridge_r = new_r
        
        self.draw_edge_on_canvas(Edge(-500, *self.current_bridge), "black", 1, (2,2), "bridge_line")

    def cleanup_final_guides(self):
        """
        方案 B (最終修正版): 在最後階段清理輔助線，並使用更可靠的邏輯校驗/移除懸空邊。
        """
        print("\n--- 清理階段：移除輔助線並校驗 Voronoi 邊 ---")
        self.canvas.delete("divider")
        self.canvas.delete("bridge_line")

        # --- 校驗邏輯開始 ---
        all_final_edges = self.left_voronoi_edges + self.right_voronoi_edges + self.merging_chain_edges
        if not any(all_final_edges):
            self.merge_phase = 'cleaned_up'
            self.btn3.config(text="完成", state=tk.DISABLED)
            print("\n圖形為空或過於簡單，無需校驗。")
            return

        endpoint_coords = []
        coord_to_vertex = {} 
        
        for edge in all_final_edges:
            if not edge: continue
            start_key = (round(edge.start.x, 3), round(edge.start.y, 3))
            end_key = (round(edge.end.x, 3), round(edge.end.y, 3))
            
            endpoint_coords.append(start_key)
            endpoint_coords.append(end_key)
            
            coord_to_vertex[start_key] = edge.start
            coord_to_vertex[end_key] = edge.end

        from collections import Counter
        endpoint_counts = Counter(endpoint_coords)

        dangling_coords = set()
        for coord, count in endpoint_counts.items():
            if count == 1:
                v = coord_to_vertex[coord]
                
                # 【關鍵修正】
                # 如果一個點在畫布的可見範圍之外，我們就認定它是延伸至無窮遠的射線端點，是合法的。
                # 只有當它在畫布範圍內，並且只連接一條邊時，才算是真正的「懸空」錯誤。
                # 增加一個小的邊界容忍值(margin)以避免浮點數誤差。
                margin = 1 
                is_outside_canvas = (
                    v.x < -margin or 
                    v.x > self.canvas_width + margin or
                    v.y < -margin or 
                    v.y > self.canvas_height + margin
                )

                if not is_outside_canvas:
                    dangling_coords.add(coord)
                    print(f"    [校驗] 發現懸空端點: {v} (位於畫布內，且只出現 {count} 次)")

        if dangling_coords:
            print(f"\n發現 {len(dangling_coords)} 個懸空端點，將移除相關的邊...")
            
            # 使用 lambda 函式簡化過濾邏輯
            is_dangling_edge = lambda edge: (
                (round(edge.start.x, 3), round(edge.start.y, 3)) in dangling_coords or
                (round(edge.end.x, 3), round(edge.end.y, 3)) in dangling_coords
            )

            original_l_count = len(self.left_voronoi_edges)
            original_r_count = len(self.right_voronoi_edges)

            self.left_voronoi_edges = [edge for edge in self.left_voronoi_edges if not is_dangling_edge(edge)]
            self.right_voronoi_edges = [edge for edge in self.right_voronoi_edges if not is_dangling_edge(edge)]

            if len(self.left_voronoi_edges) < original_l_count:
                print("    -> 已從左側(綠色)邊列表中移除懸空邊。")
            if len(self.right_voronoi_edges) < original_r_count:
                print("    -> 已從右側(紫色)邊列表中移除懸空邊。")

            self.redraw_all_edges()
        else:
            print("    [校驗] 未發現懸空邊，圖形結構完整。")
        
        self.merge_phase = 'cleaned_up'
        self.btn3.config(text="完成", state=tk.DISABLED)
        print("\n已完成輔助線清理與懸空邊校驗。")


    @property
    def current_bridge(self):
        return (self.current_bridge_l, self.current_bridge_r)
        
    def redraw_all_edges(self):
        """重繪所有邊，使用單一列表進行繪製"""
        
        print("\n" + "#"*20 + " 目前 Voronoi 邊緣狀態 " + "#"*20)
        
        print(f"\n--- 綠色邊 (左側) [{len(self.left_voronoi_edges)} 條] ---")
        if not self.left_voronoi_edges:
            print("(無)")
        else:
            for i, edge in enumerate(self.left_voronoi_edges):
                print(f"  L[{i}]: {edge}")

        print(f"\n--- 紫色邊 (右側) [{len(self.right_voronoi_edges)} 條] ---")
        if not self.right_voronoi_edges:
            print("(無)")
        else:
            for i, edge in enumerate(self.right_voronoi_edges):
                print(f"  R[{i}]: {edge}")
        
        print("#"*62 + "\n")

        print("日誌: 呼叫 redraw_all_edges -> 移除所有 'voronoi_edge' 標籤的物件...")
        self.canvas.delete("voronoi_edge")
        
        for edge in self.left_voronoi_edges: self.draw_edge_on_canvas(edge, "green", 1, tags="voronoi_edge")
        for edge in self.right_voronoi_edges: self.draw_edge_on_canvas(edge, "purple", 1, tags="voronoi_edge")
        
        print(f"【偵錯日誌】 redraw_all_edges: 準備重繪 {len(self.merging_chain_edges)} 條合併線...")
        print(f"【偵錯日誌】 merging_chain_edges 內容: {self.merging_chain_edges}")

        for i, edge in enumerate(self.merging_chain_edges):
            is_predictive = (i == len(self.merging_chain_edges) - 1)
            
            color = "red" if is_predictive else "black"
            dash = (5,3) if is_predictive and self.merge_phase != 'complete' else None
            width = 2
            
            self.draw_edge_on_canvas(edge, color, width, dash, "voronoi_edge")
            

    def draw_point(self, x, y, color="black"):
        tk_x, tk_y = self.logical_to_tk_coords(x, y)
        r = 4; self.canvas.create_oval(tk_x - r, tk_y - r, tk_x + r, tk_y + r, fill=color, outline=color)

    def on_canvas_click(self, event):
        tk_x, tk_y = event.x, event.y
        lx, ly = self.tk_to_logical_coords(tk_x, tk_y)
        if self.clicked_point_id:
            self.canvas.delete(self.clicked_point_id)
        self.clicked_point_id = self.canvas.create_oval(tk_x - 5, tk_y - 5, tk_x + 5, tk_y + 5, outline="red", width=2)
        self.coord_label.config(text=f"座標: ({lx:.1f}, {ly:.1f})")

    def read_all_test_data(self):
        self.test_data_blocks = []
        if not self.filename: 
            messagebox.showerror("錯誤", "未提供檔案名稱。")
            return
        # 嘗試用多種編碼讀取，增加通用性
        for encoding in ['utf-8', 'utf-8-sig', 'big5', 'cp950']:
            try:
                with open(self.filename, 'r', encoding=encoding) as f:
                    lines = f.readlines()
                print(f"成功使用 '{encoding}' 編碼讀取檔案。")
                break
            except Exception: 
                continue
        else: 
            messagebox.showerror("錯誤", f"無法使用任何常見編碼讀取檔案: {self.filename}")
            return
        
        i = 0
        while i < len(lines):
            try:
                line_content = lines[i].strip()
                if not line_content: 
                    i += 1
                    continue
                num_points = int(line_content)
                if num_points == 0: 
                    break
                i += 1
                block = [tuple(map(int, lines[j].strip().split())) for j in range(i, i + num_points)]
                if len(block) == num_points: 
                    self.test_data_blocks.append(block)
                i += num_points
            except (ValueError, IndexError) as e: 
                print(f"因解析錯誤跳過一個區塊: {e}")
                i += 1
        print(f"已成功從檔案載入 {len(self.test_data_blocks)} 筆測試資料。")

    def _load_and_draw_data(self):
        """根據目前的 self.current_block_index 載入並繪製資料點 (共用邏輯)"""
        self.reset_voronoi_state()
        points = self.test_data_blocks[self.current_block_index]
        for i, p in enumerate(points):
            self.draw_point(p[0], p[1], "blue")
            tk_x, tk_y = self.logical_to_tk_coords(p[0], p[1])
            self.canvas.create_text(tk_x + 8, tk_y, text=str(i), anchor="w", fill="black", font=("Arial", 9))
        print(f"\n已載入測試案例 {self.current_block_index + 1} / {len(self.test_data_blocks)}，共 {len(points)} 個點。")

    def load_next_test_data(self):
        """載入下一筆測試案例"""
        if not self.test_data_blocks:
            messagebox.showinfo("提示", "沒有可顯示的測試資料。")
            return
        self.current_block_index = (self.current_block_index + 1) % len(self.test_data_blocks)
        self._load_and_draw_data()

    def load_specific_test_data(self):
        """根據輸入框的數字，載入指定的測試案例"""
        if not self.test_data_blocks:
            messagebox.showinfo("提示", "沒有可顯示的測試資料。")
            return
        try:
            # 使用者輸入的是1-based的編號，需轉為0-based的索引
            case_num = int(self.entry_specific_case.get())
            target_index = case_num - 1

            if 0 <= target_index < len(self.test_data_blocks):
                self.current_block_index = target_index
                self._load_and_draw_data()
            else:
                messagebox.showerror("輸入無效", f"無效的案例編號。\n請輸入 1 到 {len(self.test_data_blocks)} 之間的數字。")
        except ValueError:
            messagebox.showerror("輸入無效", "請在輸入框中輸入一個有效的數字。")

if __name__ == "__main__":
    root = tk.Tk()
    # 請確認您的測資檔案名稱是否正確
    app = PointDrawerApp(root, "無註解Voronoi Diagra公開測資.txt")
    root.mainloop()