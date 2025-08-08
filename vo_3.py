# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk, messagebox
import math
import random
import sys

# 增加遞迴深度限制，以處理較多點的情況
sys.setrecursionlimit(2000)

# --- 0. 輔助資料結構 ---

class StepRecord:
    """[新增] 用於儲存演算法每一步的狀態"""
    def __init__(self, description, points, left_edges=None, right_edges=None, hyperplane=None, convex_hull=None):
        self.description = description
        self.points = [Point(p.x, p.y) for p in points]
        self.left_edges = [Edge(e.start, e.end) for e in left_edges] if left_edges else []
        self.right_edges = [Edge(e.start, e.end) for e in right_edges] if right_edges else []
        self.hyperplane = [Edge(e.start, e.end) for e in hyperplane] if hyperplane else []
        self.convex_hull = [Point(p.x, p.y) for p in convex_hull.points] if convex_hull else []

class Trend:
    """用於記錄超平面追蹤過程中的趨勢和方向"""
    def __init__(self, trend=0, direction=0, index=-1):
        self.trend, self.direction, self.index = trend, direction, index

    def __repr__(self):
        return f"Trend(T={self.trend}, D={self.direction}, I={self.index})"

    def __eq__(self, other):
        return isinstance(other, Trend) and self.trend == other.trend and \
               self.direction == other.direction and self.index == other.index

# --- 1. 幾何形狀類別 (Geometric Classes) ---

class Point:
    """代表一個 2D 平面上的點"""
    def __init__(self, x=0, y=0):
        self.x, self.y = float(x), float(y)

    def __add__(self, other): return Point(self.x + other.x, self.y + other.y)
    def __sub__(self, other): return Point(self.x - other.x, self.y - other.y)
    def __mul__(self, scalar): return Point(self.x * scalar, self.y * scalar)
    def __truediv__(self, scalar): return Point(self.x / scalar, self.y / scalar) if scalar != 0 else Point(float('inf'), float('inf'))
    def __eq__(self, other): return isinstance(other, Point) and math.isclose(self.x, other.x) and math.isclose(self.y, other.y)
    def __hash__(self): return hash((self.x, self.y))
    def __repr__(self): return f"P({self.x:.1f}, {self.y:.1f})"
    def length_sq(self): return self.x**2 + self.y**2
    def is_on_boundary(self, w, h): return math.isclose(self.x, 0) or math.isclose(self.x, w) or math.isclose(self.y, 0) or math.isclose(self.y, h)
    @staticmethod
    def cross_product(p1, p2): return p1.x * p2.y - p1.y * p2.x
    @staticmethod
    def is_similar(p1, p2, tol=1e-5): return math.isclose(p1.x, p2.x, abs_tol=tol) and math.isclose(p1.y, p2.y, abs_tol=tol)

class Edge:
    """代表一條由兩個點定義的邊"""
    def __init__(self, p1=Point(), p2=Point()):
        self.start, self.end = p1, p2
    def __repr__(self): return f"Edge({self.start}, {self.end})"
    @property
    def slope(self): return self.end - self.start

class Line(Edge):
    """代表一條無限長的線，並儲存其對應的原始站點"""
    def __init__(self, p1=Point(), p2=Point(), norm_start=None, norm_end=None):
        super().__init__(p1, p2)
        self.norm_start = norm_start if norm_start else p1
        self.norm_end = norm_end if norm_end else p2
        try:
            normal_vec = (p2 - p1).normal()
            self.a, self.b = normal_vec.x, normal_vec.y
            self.c = -1 * (self.a * self.start.x + self.b * self.start.y)
        except AttributeError:
            self.a, self.b, self.c = 0, 0, 0
    def __repr__(self): return f"Line(S={self.start}, E={self.end}, Normals=[{self.norm_start}, {self.norm_end}])"
    def to_direction(self, direction='d'):
        s = self.slope
        if (direction == 'd' and s.y < 0) or (direction == 'u' and s.y > 0) or \
           (direction == 'r' and s.x < 0) or (direction == 'l' and s.x > 0):
            self.start, self.end = self.end, self.start
    def get_perpendicular_bisector(self):
        mid = (self.norm_start + self.norm_end) / 2
        normal = Point(-(self.norm_end.y - self.norm_start.y), self.norm_end.x - self.norm_start.x)
        return Line(mid + normal * 2000, mid - normal * 2000, self.norm_start, self.norm_end)
    @staticmethod
    def intersection(l1, l2):
        """計算兩條線 (ax + by + c = 0) 的交點（修正版）。"""
        
        # delta 是係數矩陣的行列式
        delta = l1.a * l2.b - l2.a * l1.b
        
        # 如果 delta 為 0，代表兩線平行或重合，沒有唯一交點
        if math.isclose(delta, 0):
            return None
            
        # 使用克萊姆法則計算 x 和 y 的正確公式
        x = (l1.b * l2.c - l2.b * l1.c) / delta
        y = (l2.a * l1.c - l1.a * l2.c) / delta
        
        return Point(x, y)


class Polygon:
    """代表一個由頂點列表構成的凸包"""
    def __init__(self, points=None): self.points = points if points else []
    def __getitem__(self, key): return self.points[key]
    @staticmethod
    def _orientation(p, q, r):
        val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
        return 0 if math.isclose(val, 0, abs_tol=1e-6) else 1 if val > 0 else -1
    @staticmethod
    def build_hull(points):
        if len(points) <= 2: return Polygon(points)
        points.sort(key=lambda p: (p.x, p.y))
        upper, lower = [], []
        for p in points:
            while len(upper) >= 2 and Polygon._orientation(upper[-2], upper[-1], p) >= 0: upper.pop()
            upper.append(p)
        for p in reversed(points):
            while len(lower) >= 2 and Polygon._orientation(lower[-2], lower[-1], p) >= 0: lower.pop()
            lower.append(p)
        return Polygon(upper[:-1] + lower[:-1])
    @staticmethod
    def merge_convex_hulls(hull_a, hull_b):
        a_pts, b_pts = hull_a.points, hull_b.points
        a_size, b_size = len(a_pts), len(b_pts)
        idx_a, idx_b = max(range(a_size), key=lambda i: a_pts[i].x), min(range(b_size), key=lambda i: b_pts[i].x)
        upper_a, upper_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            while Polygon._orientation(b_pts[upper_b], a_pts[upper_a], a_pts[(upper_a + 1) % a_size]) >= 0: upper_a = (upper_a + 1) % a_size
            while Polygon._orientation(a_pts[upper_a], b_pts[upper_b], b_pts[(b_size + upper_b - 1) % b_size]) <= 0:
                upper_b = (b_size + upper_b - 1) % b_size
                done = False
        lower_a, lower_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            while Polygon._orientation(a_pts[lower_a], b_pts[lower_b], b_pts[(lower_b + 1) % b_size]) >= 0: lower_b = (lower_b + 1) % b_size
            while Polygon._orientation(b_pts[lower_b], a_pts[lower_a], a_pts[(a_size + lower_a - 1) % a_size]) <= 0:
                indA = (a_size + lower_a - 1) % a_size
                done = False
        return upper_a, upper_b, lower_a, lower_b

# --- 2. VORONOI 圖演算法類別 ---

class VoronoiDiagram:
    def __init__(self, points, width, height):
        self.width, self.height = width, height
        unique_points = sorted(list(set((p.x, p.y) for p in points)))
        self.points = [Point(x, y) for x, y in unique_points]
        self.steps = [] # [修改] 用於儲存每一步的狀態

    def run(self, step_mode=False):
        """主執行函式，可選擇一般模式或單步模式"""
        self.steps.clear()
        if len(self.points) < 2: return [], Polygon(), []
        
        # 在單步模式下，此函式的主要作用是填充 self.steps 列表
        edges, hull, h_planes = self._divide_and_conquer(self.points, step_mode)
        
        if not step_mode:
             # 在一般模式下，直接返回最終結果
            final_edges = self._clip_edges(edges)
            final_h_planes = self._clip_edges(h_planes)
            return final_edges, hull, final_h_planes
        # 在單步模式下，返回空值，因為結果儲存在 self.steps 中
        return None, None, None

    def _divide_and_conquer(self, points, step_mode):
        n = len(points)
        if n <= 1: return [], Polygon.build_hull(points), []

        # 基礎案例處理
        if n <= 3:
            edges, hull, h_planes = self._voronoi_base(points)
            if step_mode:
                # [關鍵修正] 在儲存步驟前，對計算出的邊進行裁剪，確保視覺化正常
                clipped_edges = self._clip_edges(edges)
                
                self.steps.append(StepRecord(f"基礎案例: 計算 {n} 個點的 Voronoi 圖", points, left_edges=clipped_edges, convex_hull=hull))
            # print(points)
            # print(edges)
            return edges, hull, h_planes

        # --- 以下遞迴與合併部分維持不變 ---
        
        # 分割
        mid = n // 2
        left_points, right_points = points[:mid], points[mid:]
        
        # 解決 (遞迴)
        left_edges, left_hull, left_hps = self._divide_and_conquer(left_points, step_mode)
        right_edges, right_hull, right_hps = self._divide_and_conquer(right_points, step_mode)

        # --- 【關鍵修正】---
        # 在合併前，先將左右子問題的 Voronoi 邊裁剪到邊界內，以確保數值穩定性。
        clipped_left_edges = self._clip_edges(left_edges)
        clipped_right_edges = self._clip_edges(right_edges)
        
        # 使用裁剪後的邊進行合併
        merged_edges, new_h_plane = self._merge(
            (clipped_left_edges, left_hull, left_points), 
            (clipped_right_edges, right_hull, right_points), 
            step_mode
        )
        
        final_hull = Polygon.build_hull(points)
        all_h_planes = left_hps + right_hps + new_h_plane
        
        if step_mode:
            # 為了讓最後一步的合併結果也顯示正確，一併進行裁剪
            clipped_final_edges = self._clip_edges(merged_edges)
            self.steps.append(StepRecord(f"完成合併: {len(points)} 個點", points, left_edges=clipped_final_edges, convex_hull=final_hull))
        
        return merged_edges, final_hull, all_h_planes
    def _voronoi_base(self, points):
        """
        處理 n=2 和 n=3 的基礎案例。
        [最終修正] 採用了更穩健的幾何向量法來確定Voronoi邊的方向，
        確保在鈍角三角形且外心極遠的情況下依然正確。
        """
        if len(points) == 2:
            p1, p2 = points[0], points[1]
            bisector = Line(p1, p2).get_perpendicular_bisector()
            hull = Polygon.build_hull(points)
            return [bisector], hull, [bisector]
        
        p1, p2, p3 = points[0], points[1], points[2]
        hull = Polygon.build_hull(points)
        edges = []

        # 檢查三點是否共線
        if abs(Point.cross_product(p2 - p1, p3 - p1)) < 1e-9:
            # 共線情況的處理邏輯... (維持不變)
            all_points = sorted([p1, p2, p3], key=lambda p: (p.x, p.y))
            edges.append(Line(all_points[0], all_points[1]).get_perpendicular_bisector())
            edges.append(Line(all_points[1], all_points[2]).get_perpendicular_bisector())
            return edges, hull, edges
        
        # 非共線（三角形）情況
        b12 = Line(p1, p2).get_perpendicular_bisector()
        b23 = Line(p2, p3).get_perpendicular_bisector()
        circumcenter = Line.intersection(b12, b23)

        if circumcenter:
            multiplier = (self.width + self.height) * 2

            # --- [全新核心邏輯] ---
            # 根據 d(X, C) > d(X, A) 的幾何原理推導出的向量判斷式

            # 處理 p1-p2 的中垂線 (參考點 p3)
            # 條件: (p1 - p3) · dir > 0
            dir12 = (p2 - p1).normal()
            ref_vec_12 = p1 - p3
            if (ref_vec_12.x * dir12.x + ref_vec_12.y * dir12.y) < 0:
                dir12 = dir12 * -1
            edges.append(Line(circumcenter, circumcenter + dir12 * multiplier, p1, p2))

            # 處理 p2-p3 的中垂線 (參考點 p1)
            # 條件: (p2 - p1) · dir > 0
            dir23 = (p3 - p2).normal()
            ref_vec_23 = p2 - p1
            if (ref_vec_23.x * dir23.x + ref_vec_23.y * dir23.y) < 0:
                dir23 = dir23 * -1
            edges.append(Line(circumcenter, circumcenter + dir23 * multiplier, p2, p3))

            # 處理 p3-p1 的中垂線 (參考點 p2)
            # 條件: (p3 - p2) · dir > 0
            dir31 = (p1 - p3).normal()
            ref_vec_31 = p3 - p2
            if (ref_vec_31.x * dir31.x + ref_vec_31.y * dir31.y) < 0:
                dir31 = dir31 * -1
            edges.append(Line(circumcenter, circumcenter + dir31 * multiplier, p3, p1))
            # --- [全新邏輯結束] ---
            
        else:
            # 幾乎共線但未被攔截的退化情況
            edges.append(b12)
            edges.append(b23)
        
        return edges, hull, edges

    def _merge(self, left_data, right_data, step_mode):
        left_edges, left_hull, left_points = left_data
        right_edges, right_hull, right_points = right_data
        
        ua_idx, ub_idx, la_idx, lb_idx = Polygon.merge_convex_hulls(left_hull, right_hull)
        p_ua, p_ub = left_hull[ua_idx], right_hull[ub_idx]
        p_la, p_lb = left_hull[la_idx], right_hull[lb_idx]
        
        if step_mode:
            merged_hull = Polygon.build_hull(left_points + right_points)
            self.steps.append(StepRecord("步驟 1/4: 合併凸包", left_points + right_points, left_edges, right_edges, convex_hull=merged_hull))

        hyperplane, scan_trends = [], []
        scan_line = Line(p_ua, p_ub)
        lower_tangent = Line(p_la, p_lb)
        
        l_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in left_edges]
        r_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in right_edges]

        while True:
            scan_pb = scan_line.get_perpendicular_bisector()
            scan_pb.to_direction('d')
            if hyperplane and not hyperplane[-1].end.is_on_boundary(self.width, self.height):
                scan_pb.start = hyperplane[-1].end

            l_is_pt, l_is_idx = self._find_near_intra_is(scan_pb, l_edges)
            r_is_pt, r_is_idx = self._find_near_intra_is(scan_pb, r_edges)
            near_is_pt, choose_dir, near_is_idx = self._find_near_inter_is(scan_pb, l_is_pt, r_is_pt, l_is_idx, r_is_idx)
            scan_trends.append(Trend(direction=choose_dir, index=near_is_idx))
            
            is_lower = Point.is_similar(scan_line.norm_start, lower_tangent.norm_start) and Point.is_similar(scan_line.norm_end, lower_tangent.norm_end)
            if near_is_pt and not is_lower:
                scan_pb.end = near_is_pt
            else:
                 clipped = self._clip_edges([scan_pb])
                 if clipped: scan_pb = clipped[0]
            hyperplane.append(scan_pb)

            if is_lower or not near_is_pt: break
            scan_line = self._move_scan_next_loop(scan_line, l_edges, r_edges, l_is_idx, r_is_idx, choose_dir)

        if step_mode:
            self.steps.append(StepRecord("步驟 2/4: 追蹤超平面", left_points + right_points, l_edges, r_edges, hyperplane))
        
        self._update_trend(hyperplane, scan_trends)
        self._elimination(hyperplane, scan_trends, l_edges, r_edges)
        
        if step_mode:
            self.steps.append(StepRecord("步驟 3/4: 執行消線", left_points + right_points, l_edges, r_edges, hyperplane))

        final_edges = l_edges + r_edges + hyperplane
        return final_edges, hyperplane

    def _find_near_intra_is(self, scan_pb, edges):
        nearest_is_pt, nearest_is_idx, min_dist_sq = None, -1, float('inf')
        scan_slope = scan_pb.slope
        for i, edge in enumerate(edges):
            if not (scan_pb.norm_start == edge.norm_start or scan_pb.norm_start == edge.norm_end or \
                    scan_pb.norm_end == edge.norm_start or scan_pb.norm_end == edge.norm_end):
                continue
            is_pt = Line.intersection(scan_pb, edge)
            if is_pt and not Point.is_similar(is_pt, scan_pb.start):
                vec_to_is = is_pt - scan_pb.start
                if (vec_to_is.x * scan_slope.x + vec_to_is.y * scan_slope.y) > 1e-9:
                    dist_sq = vec_to_is.length_sq()
                    if dist_sq < min_dist_sq:
                        min_dist_sq, nearest_is_pt, nearest_is_idx = dist_sq, is_pt, i
        return nearest_is_pt, nearest_is_idx
    
    def _find_near_inter_is(self, scan_pb, l_pt, r_pt, l_idx, r_idx):
        if l_pt and r_pt:
            dist_l_sq = (l_pt - scan_pb.start).length_sq()
            dist_r_sq = (r_pt - scan_pb.start).length_sq()
            if math.isclose(dist_l_sq, dist_r_sq): return None, 0, -1
            return (l_pt, 1, l_idx) if dist_l_sq < dist_r_sq else (r_pt, 2, r_idx)
        return (l_pt, 1, l_idx) if l_pt else (r_pt, 2, r_idx) if r_pt else (None, 0, -1)

    def _move_scan_next_loop(self, scan_line, l_edges, r_edges, l_idx, r_idx, choose):
        new_start, new_end = scan_line.norm_start, scan_line.norm_end
        if choose == 1:
            edge = l_edges[l_idx]
            new_start = edge.norm_end if new_start == edge.norm_start else edge.norm_start
        elif choose == 2:
            edge = r_edges[r_idx]
            new_end = edge.norm_end if new_end == edge.norm_start else edge.norm_start
        return Line(new_start, new_end)

    def _update_trend(self, hyperplane, scan_trends):
        for i in range(len(hyperplane) - 1):
            cp = Point.cross_product(hyperplane[i].slope, hyperplane[i+1].slope)
            if cp > 1e-9: scan_trends[i].trend = 1
            elif cp < -1e-9: scan_trends[i].trend = 2
    
    def _is_same_trend(self, scan_trends, j):
        for i in range(j):
            if scan_trends[i] == scan_trends[j]: return True
        return False

    def _remove_dangling_line(self, voronoi_edges, del_point):
        count, index_to_remove = 0, -1
        for i, edge in enumerate(voronoi_edges):
            if edge.start == del_point or edge.end == del_point:
                count += 1
                index_to_remove = i
        if count == 1 and index_to_remove != -1: voronoi_edges.pop(index_to_remove)

    def _elimination(self, hyperplane, scan_trends, l_edges, r_edges):
        for i in range(len(scan_trends) - 1):
            trend_info = scan_trends[i]
            edges_list = l_edges if trend_info.direction == 1 else r_edges if trend_info.direction == 2 else None
            if not edges_list or not (0 <= trend_info.index < len(edges_list)): continue
            edge_to_prune, intersection_pt = edges_list[trend_info.index], hyperplane[i].end
            if intersection_pt.is_on_boundary(self.width, self.height): continue
            
            if trend_info.trend == 1: # 逆時針
                if edge_to_prune.end.is_on_boundary(self.width, self.height) or self._is_same_trend(scan_trends, i):
                    edge_to_prune.start = intersection_pt
                else:
                    old_endpoint = Point(edge_to_prune.end.x, edge_to_prune.end.y)
                    edge_to_prune.end = intersection_pt
                    self._remove_dangling_line(edges_list, old_endpoint)
            elif trend_info.trend == 2: # 順時針
                if edge_to_prune.start.is_on_boundary(self.width, self.height) or self._is_same_trend(scan_trends, i):
                    edge_to_prune.end = intersection_pt
                else:
                    old_startpoint = Point(edge_to_prune.start.x, edge_to_prune.start.y)
                    edge_to_prune.start = intersection_pt
                    self._remove_dangling_line(edges_list, old_startpoint)

    def _clip_edges(self, edges):
        clipped = []
        for edge in edges:
            p1_copy, p2_copy = Point(edge.start.x, edge.start.y), Point(edge.end.x, edge.end.y)
            clipped_edge = self._cohen_sutherland_clip(p1_copy, p2_copy)
            if clipped_edge:
                p1, p2 = clipped_edge
                if not Point.is_similar(p1, p2):
                    new_edge = Line(p1, p2, getattr(edge, 'norm_start', None), getattr(edge, 'norm_end', None))
                    clipped.append(new_edge)
        return clipped
    
    def _cohen_sutherland_clip(self, p1, p2):
        INSIDE, LEFT, RIGHT, BOTTOM, TOP = 0, 1, 2, 4, 8
        def _get_code(p):
            code = INSIDE
            if p.x < 0: code |= LEFT
            elif p.x > self.width: code |= RIGHT
            if p.y < 0: code |= BOTTOM
            elif p.y > self.height: code |= TOP
            return code
        code1, code2 = _get_code(p1), _get_code(p2)
        while True:
            if not (code1 | code2): return (p1, p2)
            if code1 & code2: return None
            code_out = code1 or code2
            x, y = 0.0, 0.0
            dy, dx = p2.y - p1.y, p2.x - p1.x
            if code_out & TOP: x, y = p1.x + dx * (self.height - p1.y) / dy if dy != 0 else p1.x, self.height
            elif code_out & BOTTOM: x, y = p1.x + dx * (0 - p1.y) / dy if dy != 0 else p1.x, 0
            elif code_out & RIGHT: y, x = p1.y + dy * (self.width - p1.x) / dx if dx != 0 else p1.y, self.width
            elif code_out & LEFT: y, x = p1.y + dy * (0 - p1.x) / dx if dx != 0 else p1.y, 0
            if code_out == code1: p1, code1 = Point(x, y), _get_code(Point(x, y))
            else: p2, code2 = Point(x, y), _get_code(Point(x, y))

# --- 3. 應用程式 GUI 類別 ---

class Application(tk.Tk):
    def __init__(self, width=600, height=600):
        super().__init__()
        self.width, self.height = width, height
        self.title("Voronoi 圖產生器 (左下角為原點)")
        self.geometry(f"{width + 250}x{height + 50}")
        self.resizable(False, False)
        self.points, self.voronoi_steps, self.current_step = [], [], 0
        self._create_widgets()

    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main_frame, width=self.width, height=self.height, bg="white", highlightthickness=1, highlightbackground="black")
        self.canvas.pack(side=tk.LEFT, padx=(0, 10), fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.add_point_on_click)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # --- 點控制 ---
        point_frame = ttk.LabelFrame(control_frame, text="1. 新增站點", padding=10)
        point_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(point_frame, text="隨機產生點數：").pack()
        self.random_points_entry = ttk.Entry(point_frame)
        self.random_points_entry.pack(fill=tk.X, pady=(0, 5))
        self.random_points_entry.insert(0, "3")
        ttk.Button(point_frame, text="隨機產生", command=self.generate_random_points).pack(fill=tk.X)
        ttk.Button(point_frame, text="清除畫布", command=self.clear_canvas).pack(fill=tk.X, pady=(5,0))

        # --- 執行控制 ---
        run_frame = ttk.LabelFrame(control_frame, text="2. 執行演算法", padding=10)
        run_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(run_frame, text="執行 (一次完成)", command=self.run_normal).pack(fill=tk.X)
        ttk.Button(run_frame, text="產生步驟", command=self.run_step_by_step_generate).pack(fill=tk.X, pady=5)
        
        # --- 單步控制 ---
        step_frame = ttk.LabelFrame(control_frame, text="3. 單步執行控制", padding=10)
        step_frame.pack(fill=tk.X)
        
        nav_frame = ttk.Frame(step_frame)
        nav_frame.pack(fill=tk.X)
        self.prev_button = ttk.Button(nav_frame, text="上一步", command=self.prev_step, state=tk.DISABLED)
        self.prev_button.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self.next_button = ttk.Button(nav_frame, text="下一步", command=self.next_step, state=tk.DISABLED)
        self.next_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5,0))

        self.step_label = ttk.Label(step_frame, text="尚未產生步驟", anchor="center")
        self.step_label.pack(fill=tk.X, pady=5)

    def add_point_on_click(self, event):
        # [MODIFIED] Convert y-coordinate from top-left to bottom-left origin
        self.points.append(Point(event.x, self.height - event.y))
        self.reset_steps()
        self.draw_points()

    def generate_random_points(self):
        self.clear_canvas()
        try:
            num = int(self.random_points_entry.get())
            # Random points are generated within the correct coordinate space already
            for _ in range(num): self.points.append(Point(random.randint(10, self.width - 10), random.randint(10, self.height - 10)))
        except (ValueError, TypeError): messagebox.showerror("無效輸入", "請輸入一個有效的整數。")
        self.draw_points()

    def run_normal(self):
        """執行一次完成的模式"""
        if len(self.points) < 2: return
        self.reset_steps()
        voronoi = VoronoiDiagram(self.points, self.width, self.height)
        
        # 在一般模式下，run() 會返回最終計算和裁剪後的邊
        edges, hull, h_planes = voronoi.run(step_mode=False)
        
        # 為了繪製最終結果，我們將所有邊合併到一個列表中
        all_final_edges = edges # h_planes are already part of edges in the new logic
        
        self.draw_all(
            points=self.points,
            left_edges=all_final_edges, # 將所有邊作為主邊傳入
            right_edges=[],             # 其他列表留空
            hyperplane=[],
            convex_hull_pts=hull.points if hull else [],
            is_final_run=True           # 設定為最終執行模式
        )

    def run_step_by_step_generate(self):
        if len(self.points) < 2: return
        self.reset_steps()
        voronoi = VoronoiDiagram(self.points, self.width, self.height)
        voronoi.run(step_mode=True)
        self.voronoi_steps = voronoi.steps
        if self.voronoi_steps:
            self.current_step = 0
            self.update_step_display()
            self.prev_button.config(state=tk.NORMAL)
            self.next_button.config(state=tk.NORMAL)

    def next_step(self):
        if self.current_step < len(self.voronoi_steps) - 1:
            self.current_step += 1
            self.update_step_display()

    def prev_step(self):
        if self.current_step > 0:
            self.current_step -= 1
            self.update_step_display()

    def update_step_display(self):
        """更新單步執行的畫面顯示"""
        if not self.voronoi_steps: return
        step_record = self.voronoi_steps[self.current_step]
        self.step_label.config(text=f"第 {self.current_step + 1}/{len(self.voronoi_steps)} 步\n{step_record.description}")
        
        # 在單步模式下，呼叫 draw_all 並指明這不是最終執行
        self.draw_all(
            points=step_record.points, 
            left_edges=step_record.left_edges, 
            right_edges=step_record.right_edges, 
            hyperplane=step_record.hyperplane, 
            convex_hull_pts=step_record.convex_hull,
            is_final_run=False # 明確設定為 False
        )
        
        self.prev_button.config(state=tk.NORMAL if self.current_step > 0 else tk.DISABLED)
        self.next_button.config(state=tk.NORMAL if self.current_step < len(self.voronoi_steps) - 1 else tk.DISABLED)

    def clear_canvas(self):
        self.points.clear()
        self.reset_steps()
        self.canvas.delete("all")

    def reset_steps(self):
        self.voronoi_steps.clear()
        self.current_step = 0
        self.step_label.config(text="尚未產生步驟")
        self.prev_button.config(state=tk.DISABLED)
        self.next_button.config(state=tk.DISABLED)

    def draw_points(self):
        self.canvas.delete("all")
        for p in self.points:
            # [MODIFIED] Convert y-coordinate for drawing
            y = self.height - p.y
            self.canvas.create_oval(p.x - 3, y - 3, p.x + 3, y + 3, fill="red", outline="black")

    def draw_all(self, points, left_edges, right_edges, hyperplane, convex_hull_pts, is_final_run=False):
        """
        [MODIFIED] Unified drawing function, converts all y-coordinates for display.
        """
        self.canvas.delete("all")

        # Helper function to convert coordinates for drawing
        def convert_y(y_coord):
            return self.height - y_coord

        if is_final_run:
            # Final mode: all edges are drawn in black
            for edge in left_edges:
                self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="black", width=1.5)
        else:
            # Step-by-step mode: different colors
            for edge in left_edges: self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="#a6d1e6", width=1.5)
            for edge in right_edges: self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="#b5e5a4", width=1.5)
            for edge in hyperplane: self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="blue", width=2)
        
        # Draw convex hull
        if len(convex_hull_pts) > 1:
            # [MODIFIED] Convert y-coordinates for hull points
            flat_pts = [c for p in convex_hull_pts for c in (p.x, convert_y(p.y))]
            if len(convex_hull_pts) == 2:
                self.canvas.create_line(flat_pts, fill="green", width=2)
            else:
                self.canvas.create_polygon(flat_pts, outline="green", fill="", width=2)
        
        # Draw sites (points)
        for p in points:
            y = convert_y(p.y)
            self.canvas.create_oval(p.x - 3, y - 3, p.x + 3, y + 3, fill="red", outline="black")

if __name__ == "__main__":
    Point.normal = lambda self: Point(-self.y, self.x)
    app = Application()
    app.mainloop()