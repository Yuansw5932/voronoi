import tkinter as tk
from tkinter import messagebox
import math
from collections import Counter
import time
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
        self.left_half_points = []
        self.right_half_points = []
        self.sorted_points=[]
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
        if not vertices: return []
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
        if not all([edge1, edge2, edge1.start, edge1.end, edge2.start, edge2.end]):
            return None
            
        p1, p2 = edge1.start, edge1.end
        p3, p4 = edge2.start, edge2.end

        den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x)
        if abs(den) < 1e-9:
            return None

        t_num = (p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)
        u_num = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x))

        t = t_num / den
        u = u_num / den

        if check_segment1 and not (0.0 <= t <= 1.0):
            return None
        if check_segment2 and not (0.0 <= u <= 1.0):
            return None

        x = p1.x + t * (p2.x - p1.x)
        y = p1.y + t * (p2.y - p1.y)
        
        return Vertex(id=-10, x=x, y=y)

    def get_three_line_cross_point(self, v1, v2, v3):
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
                l_idx = l_cand_idx
                changed = True
            
            r_cand_idx = (r_idx + (1 if find_upper else -1) + len(right_hull)) % len(right_hull)
            if ((find_upper and self.cross_product(left_hull[l_idx], right_hull[r_idx], right_hull[r_cand_idx]) > 0) or
               (not find_upper and self.cross_product(left_hull[l_idx], right_hull[r_idx], right_hull[r_cand_idx]) < 0)):
                r_idx = r_cand_idx
                changed = True

            if not changed:
                break
        return left_hull[l_idx], right_hull[r_idx]

    def process_and_draw_points(self):
        if self.current_block_index == -1: 
            messagebox.showinfo("提示", "請先載入測試資料。")
            return
            
        self.reset_voronoi_state() 
        points = self.test_data_blocks[self.current_block_index]
        vertices = [Vertex(id=i, x=p[0], y=p[1]) for i, p in enumerate(points)]

        for v in vertices: 
            self.draw_point(v.x, v.y, "blue")
            self.canvas.create_text(*self.logical_to_tk_coords(v.x, v.y), text=str(v.id), anchor="n", fill="black", font=("Arial", 9))

        if len(vertices) <= 3:
            # 如果點太少，直接計算並顯示結果
            self.left_voronoi_edges = self.compute_voronoi_for_small_set(vertices)
            self.redraw_all_edges()
            self.merge_phase = 'complete'
            self.btn3.config(text="清理輔助線")
            return

        x_sorted = sorted(vertices, key=lambda v: (v.x, v.y))
        mid = len(x_sorted) // 2
        
        self.left_half_points, self.right_half_points = x_sorted[:mid], x_sorted[mid:]
        self.sorted_points = x_sorted
        
        divider_x = (self.left_half_points[-1].x + self.right_half_points[0].x) / 2.0
        self.draw_edge_on_canvas(Edge(-100, Vertex(-98, divider_x, 0), Vertex(-99, divider_x, 600)), "blue", 2, (5, 3), "divider")
        
        self.merge_phase = 'ready_to_merge'
        self.btn3.config(text="計算子問題")
        print("已完成分割與排序，準備遞迴計算子問題。")
           
    def next_step_action(self):
        actions = {
            'idle': lambda: messagebox.showinfo("提示", "請先點擊「分割與排序」。"),
            'ready_to_merge': self.draw_initial_bridge,
            'bridge_drawn': self.start_merging_chain,
            'merging': self.merge_voronoi_step,
            'complete': self.cleanup_final_guides,
            'cleaned_up': lambda: messagebox.showinfo("提示", "已清理輔助線。請載入下一個測試案例。")
        }
        actions.get(self.merge_phase, lambda: None)()

    # ----- 新增：正確的遞迴函式 -----
    def _solve_voronoi_recursively(self, vertices):
        """
        一個純計算的遞迴函式(黑盒子)。
        輸入一群點，回傳計算好的 Voronoi Edges 和 Convex Hull。
        """
        if len(vertices) <= 3:
            edges = self.compute_voronoi_for_small_set(vertices)
            hull = self.sort_vertices_by_angle(vertices)
            return edges, hull

        mid = len(vertices) // 2
        left_points, right_points = vertices[:mid], vertices[mid:]

        left_edges, left_hull = self._solve_voronoi_recursively(left_points)
        right_edges, right_hull = self._solve_voronoi_recursively(right_points)
        
        # 使用一個批次處理的合併函式
        merged_edges = self._merge_diagrams_batch(left_edges, left_hull, right_edges, right_hull)
        
        merged_hull = self.sort_vertices_by_angle(vertices)
        return merged_edges, merged_hull

    # ----- 新增：批次處理的合併函式 -----
    def _merge_diagrams_batch(self, left_edges, left_hull, right_edges, right_hull):
        """
        給定左右子圖，一次性計算出合併結果，不進行互動。
        """
        upper_l, upper_r = self.find_common_tangent(left_hull, right_hull, find_upper=True)
        lower_l, lower_r = self.find_common_tangent(left_hull, right_hull, find_upper=False)

        if not upper_l or not lower_l:
            return left_edges + right_edges

        # 處理邊界情況：上下公切線相同
        if upper_l.id == lower_l.id and upper_r.id == lower_r.id:
            return left_edges + right_edges + [self.get_perpendicular_bisector(upper_l, upper_r, -99)]

        merging_chain = []
        bridge_l, bridge_r = upper_l, upper_r
        last_intersect = None

        while True:
            current_bisector = self.get_perpendicular_bisector(bridge_l, bridge_r, -1)
            event = self.find_next_event(current_bisector, left_edges, right_edges, point_to_ignore=last_intersect)
            intersect_pt = event.get('intersect_pt')

            if not intersect_pt:
                # 找不到交點，從 last_intersect 延伸射線後結束
                if last_intersect:
                    dx, dy = current_bisector.end.x - current_bisector.start.x, current_bisector.end.y - current_bisector.start.y
                    if (bridge_l.y + bridge_r.y)/2 < (lower_l.y + lower_r.y)/2: # 往下走
                         if dy > 0: dx, dy = -dx, -dy
                    else: # 往上走
                         if dy < 0: dx, dy = -dx, -dy
                    ray_end = Vertex(-1, last_intersect.x + dx*2000, last_intersect.y + dy*2000)
                    merging_chain.append(Edge(-1, last_intersect, ray_end, origin_vertices=(bridge_l, bridge_r)))
                break

            # 建立合併邊
            if not last_intersect: # 第一條邊
                dx, dy = current_bisector.end.x - current_bisector.start.x, current_bisector.end.y - current_bisector.start.y
                if dy < 0: dx, dy = -dx, -dy # 確保向上
                ray_start = Vertex(-1, intersect_pt.x - dx*2000, intersect_pt.y - dy*2000)
                merging_chain.append(Edge(-1, ray_start, intersect_pt, origin_vertices=(bridge_l, bridge_r)))
            else: # 中間的邊
                merging_chain.append(Edge(-1, last_intersect, intersect_pt, origin_vertices=(bridge_l, bridge_r)))
            
            last_intersect = intersect_pt
            
            # 如果到達下公切線，結束迴圈
            if bridge_l.id == lower_l.id and bridge_r.id == lower_r.id:
                break

            # 更新橋樑和修剪邊
            edge_to_trim, side = event['edge'], event['side']
            self.update_bridge_and_trim(event, (bridge_l, bridge_r), is_interactive=False)
            
            if side == 'left':
                bridge_l = next(v for v in edge_to_trim.origin_vertices if v.id != bridge_l.id)
            else: # right
                bridge_r = next(v for v in edge_to_trim.origin_vertices if v.id != bridge_r.id)

        return left_edges + right_edges + merging_chain

    def draw_initial_bridge(self):
        print("\n--- 步驟：計算子問題並繪製初始橋樑 ---")
        self.canvas.delete("voronoi_edge", "bridge_line")
        
        # ----- 主要修改點 -----
        # 呼叫新的遞迴函式來正確、完整地計算左右子圖
        print("    -> 正在遞迴計算左半邊...")
        self.left_voronoi_edges, self.left_hull = self._solve_voronoi_recursively(self.left_half_points)
       
        print("    -> 正在遞迴計算右半邊...")
        self.right_voronoi_edges, self.right_hull = self._solve_voronoi_recursively(self.right_half_points)
        print("子問題計算完成。")
        # ----- 修改結束 -----
        
        self.upper_tangent = self.find_common_tangent(self.left_hull, self.right_hull, True)
        self.lower_tangent = self.find_common_tangent(self.left_hull, self.right_hull, False)
        
        if not self.upper_tangent[0] or not self.upper_tangent[1]:
            self.merge_phase = 'complete'
            self.btn3.config(text="清理輔助線")
            self.redraw_all_edges()
            return
            
        self.current_bridge_l, self.current_bridge_r = self.upper_tangent
        print(f"上切線 (初始橋樑): {self.current_bridge_l} -> {self.current_bridge_r}")
        self.draw_edge_on_canvas(Edge(-500, *self.current_bridge), "black", 1, (2,2), "bridge_line")
        
        self.merge_phase = 'bridge_drawn'
        self.btn3.config(text="繪製第一條合併線")
        self.redraw_all_edges()

    def start_merging_chain(self):
        print("\n--- 步驟：繪製第一條合併線 ---")
        initial_bisector = self.get_perpendicular_bisector(*self.current_bridge, -201)
        
        first_event = self.find_next_event(initial_bisector, self.left_voronoi_edges, self.right_voronoi_edges, point_to_ignore=None)
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

        color = "red" 
        dash = (5,3) 
        width = 2
        self.draw_edge_on_canvas(edge, color, width, dash, "hi")

        self.merging_chain_edges.append(edge)
        self.update_bridge_and_trim(first_event, self.current_bridge, is_interactive=True)
        self.merge_phase = 'merging'
        self.btn3.config(text="下一步")
        self.redraw_all_edges()


    def merge_voronoi_step(self):
        if self.merge_phase != 'merging' or not self.merging_chain_edges: return
        
        print(f"\n--- 步驟 {len(self.merging_chain_edges) + 2}：尋找下一個交點 ---")

        if self.current_bridge_l == self.lower_tangent[0] and self.current_bridge_r == self.lower_tangent[1]:
             print("橋樑已抵達下公切線，合併完成！")
             last_ray = self.merging_chain_edges[-1]
             p_bisector = self.get_perpendicular_bisector(*self.lower_tangent, -999)
             final_intersection = self.get_line_intersection(last_ray, p_bisector, check_segment1=True, check_segment2=False)
             if final_intersection: last_ray.end = final_intersection
             
             for i, edge in enumerate(self.merging_chain_edges):
                is_predictive = (i == len(self.merging_chain_edges) - 1)
                
                color = "red" if is_predictive else "black"
                dash = (5,3) if is_predictive and self.merge_phase != 'complete' else None
                width = 2
                self.draw_edge_on_canvas(edge, color, width, dash, "hi")


             self.merge_phase = 'complete'
             self.btn3.config(text="清理輔助線")
             self.redraw_all_edges()
             return

        last_known_intersection = self.merging_chain_edges[-1].start
        current_bisector = self.get_perpendicular_bisector(*self.current_bridge, -202)
        next_event = self.find_next_event(current_bisector, self.left_voronoi_edges, self.right_voronoi_edges, point_to_ignore=last_known_intersection)
        next_intersect = next_event.get('intersect_pt')
        
        if not next_intersect:
            print("找不到新的交點，延伸最後射線並結束。")
            last_edge = self.merging_chain_edges[-1]
            dx, dy = current_bisector.end.x - current_bisector.start.x, current_bisector.end.y - current_bisector.start.y
            if dy > 0: dx, dy = -dx, -dy
            last_edge.end = Vertex(-1, last_edge.start.x + dx * 1000, last_edge.start.y + dy * 1000)
            self.merge_phase = 'complete'
            self.btn3.config(text="清理輔助線")
            self.redraw_all_edges()
            return
        
        self.merging_chain_edges[-1].end = next_intersect
        self.update_bridge_and_trim(next_event, self.current_bridge, is_interactive=True)

        new_bisector = self.get_perpendicular_bisector(*self.current_bridge, -203)
        dx, dy = new_bisector.end.x - new_bisector.start.x, new_bisector.end.y - new_bisector.start.y
        if dy > 0: dx, dy = -dx, -dy
        ray_end = Vertex(-1, next_intersect.x + dx * 1000, next_intersect.y + dy * 1000)
        self.merging_chain_edges.append(Edge(-204, next_intersect, ray_end, origin_vertices=self.current_bridge))
        self.redraw_all_edges()

    def find_next_event(self, bisector_line, left_edges, right_edges, point_to_ignore=None):
        events = []
        ignore_tolerance_sq = 1e-9

        for edge in left_edges:
            if edge and edge.origin_vertices:
                pt = self.get_line_intersection(bisector_line, edge, check_segment1=False, check_segment2=True)
                if pt and (not point_to_ignore or self.distance_sq(pt, point_to_ignore) > ignore_tolerance_sq):
                    events.append({'intersect_pt': pt, 'edge': edge, 'side': 'left'})
        
        for edge in right_edges:
            if edge and edge.origin_vertices:
                pt = self.get_line_intersection(bisector_line, edge, check_segment1=False, check_segment2=True)
                if pt and (not point_to_ignore or self.distance_sq(pt, point_to_ignore) > ignore_tolerance_sq):
                    events.append({'intersect_pt': pt, 'edge': edge, 'side': 'right'})
        
        if not events:
            return {}
        
        best_event = max(events, key=lambda e: e['intersect_pt'].y - abs(e['intersect_pt'].x) * 1e-9)
        return best_event

    def update_bridge_and_trim(self, event, current_bridge_tuple, is_interactive=True):
        edge, side, pt = event.get('edge'), event.get('side'), event.get('intersect_pt')
        if not all([edge, side, pt]): return

        bridge_l, bridge_r = current_bridge_tuple

        if side == 'right':
            is_invalid = lambda p: self.distance_sq(p, bridge_l) < self.distance_sq(p, bridge_r)
        else:
            is_invalid = lambda p: self.distance_sq(p, bridge_r) < self.distance_sq(p, bridge_l)

        if is_invalid(edge.start): edge.start = pt
        elif is_invalid(edge.end): edge.end = pt
        
        if is_interactive:
            self.canvas.delete("bridge_line")
            if side == 'left':
                new_l = next(v for v in edge.origin_vertices if v != self.current_bridge_l)
                self.current_bridge_l = new_l
            else:
                new_r = next(v for v in edge.origin_vertices if v != self.current_bridge_r)
                self.current_bridge_r = new_r
            self.draw_edge_on_canvas(Edge(-500, *self.current_bridge), "black", 1, (2,2), "bridge_line")

    def cleanup_final_guides(self):
        self.canvas.delete("divider", "bridge_line")
        all_final_edges = self.left_voronoi_edges + self.right_voronoi_edges + self.merging_chain_edges
        
        # 此處的清理邏輯可以保持不變
        self.merge_phase = 'cleaned_up'
        self.btn3.config(text="完成", state=tk.DISABLED)
        self.redraw_all_edges()


    @property
    def current_bridge(self):
        return (self.current_bridge_l, self.current_bridge_r)
        
    def redraw_all_edges(self):
        self.canvas.delete("voronoi_edge")
        
        for edge in self.left_voronoi_edges: self.draw_edge_on_canvas(edge, "green", 1, tags="voronoi_edge")
        for edge in self.right_voronoi_edges: self.draw_edge_on_canvas(edge, "purple", 1, tags="voronoi_edge")
        
        for i, edge in enumerate(self.merging_chain_edges):
            is_predictive = (i == len(self.merging_chain_edges) - 1) and self.merge_phase == 'merging'
            color = "red" if is_predictive else "black"
            dash = (5,3) if is_predictive else None
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
                    i += 1; continue
                num_points = int(line_content)
                if num_points == 0: break
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
        self.reset_voronoi_state()
        points = self.test_data_blocks[self.current_block_index]
        for i, p in enumerate(points):
            self.draw_point(p[0], p[1], "blue")
            tk_x, tk_y = self.logical_to_tk_coords(p[0], p[1])
            self.canvas.create_text(tk_x + 8, tk_y, text=str(i), anchor="w", fill="black", font=("Arial", 9))

    def load_next_test_data(self):
        if not self.test_data_blocks:
            messagebox.showinfo("提示", "沒有可顯示的測試資料。")
            return
        self.current_block_index = (self.current_block_index + 1) % len(self.test_data_blocks)
        self._load_and_draw_data()

    def load_specific_test_data(self):
        if not self.test_data_blocks:
            messagebox.showinfo("提示", "沒有可顯示的測試資料。")
            return
        try:
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
    app = PointDrawerApp(root, "無註解Voronoi Diagra公開測資.txt")
    root.mainloop()