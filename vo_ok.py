# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk, messagebox
import math
import random
import sys

# 增加遞迴深度限制，以處理較多點的情況
sys.setrecursionlimit(2000)

# --- 0. 輔助資料結構 ---

class Trend:
    """用於記錄超平面追蹤過程中的趨勢和方向，對應 C++ 中的 Trend 結構。"""
    def __init__(self, trend=0, direction=0, index=-1):
        self.trend = trend      # 0:無, 1:逆時針, 2:順時針
        self.direction = direction  # 0:無, 1:左側, 2:右側
        self.index = index      # -1:無 (相交邊的索引)
    def __repr__(self):
        return f"Trend(T={self.trend}, D={self.direction}, I={self.index})"

# --- 1. 幾何形狀類別 (Geometric Classes) ---

class Point:
    """代表一個 2D 平面上的點，具有整數座標。"""
    def __init__(self, x=0, y=0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)
        
    def __mul__(self, scalar):
        return Point(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        # 避免除以零的錯誤
        if scalar == 0:
            return Point(float('inf'), float('inf'))
        return Point(self.x / scalar, self.y / scalar)
        
    def __eq__(self, other):
        if not isinstance(other, Point): return NotImplemented
        # 使用一個小的容錯值來比較浮點數
        return math.isclose(self.x, other.x) and math.isclose(self.y, other.y)
        
    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        # 讓 Point 物件可以被加入 set 或作為字典的鍵
        return hash((self.x, self.y))

    def __repr__(self):
        return f"P({self.x:.2f}, {self.y:.2f})"

    def length_sq(self):
        """計算到原點距離的平方，避免開根號，用於比較大小。"""
        return self.x**2 + self.y**2

    def is_on_boundary(self, width, height):
        """檢查點是否在畫布的邊界上。"""
        return math.isclose(self.x, 0) or math.isclose(self.x, width) or \
               math.isclose(self.y, 0) or math.isclose(self.y, height)
    
    @staticmethod
    def cross_product(p1, p2):
        return p1.x * p2.y - p1.y * p2.x

class Edge:
    """代表一條由兩個點定義的邊。"""
    def __init__(self, p1=Point(), p2=Point()):
        self.start = p1
        self.end = p2

    def __repr__(self):
        return f"Edge({self.start}, {self.end})"
    
    @property
    def slope(self):
        return self.end - self.start

class Line(Edge):
    """
    代表一條無限長的線 (ax + by + c = 0)，同時也儲存其對應的原始站點。
    """
    def __init__(self, p1=Point(), p2=Point(), norm_start=None, norm_end=None):
        super().__init__(p1, p2)
        # norm_start 和 norm_end 是定義這條線作為垂直平分線的兩個原始站點
        self.norm_start = norm_start if norm_start else p1
        self.norm_end = norm_end if norm_end else p2
        
        # 線的方程式 ax + by + c = 0
        normal_vec = (p2 - p1).normal()
        self.a = normal_vec.x
        self.b = normal_vec.y
        self.c = -1 * (self.a * self.start.x + self.b * self.start.y)

    def __repr__(self):
        return f"Line(S={self.start}, E={self.end})"

    def to_direction(self, direction='d'):
        """根據方向調整線段的起點和終點。"""
        s = self.slope
        if (direction == 'd' and s.y < 0) or (direction == 'r' and s.x < 0):
            self.start, self.end = self.end, self.start

    def get_perpendicular_bisector(self):
        """計算這條線段的垂直平分線。"""
        mid_point = (self.norm_start + self.norm_end) / 2
        slope_vec = self.norm_end - self.norm_start
        normal_vec = Point(-slope_vec.y, slope_vec.x)
        
        # 創建一條沿著平分線的長線
        p1 = mid_point + normal_vec * 2000
        p2 = mid_point - normal_vec * 2000
        
        return Line(p1, p2, norm_start=self.norm_start, norm_end=self.norm_end)

    @staticmethod
    def intersection(l1, l2):
        delta = l1.a * l2.b - l2.a * l1.b
        if math.isclose(delta, 0):
            return None # 平行線
        
        delta_x = -l1.c * l2.b - (-l2.c * l1.b)
        delta_y = l1.a * (-l2.c) - (-l1.c * l2.a)
        
        return Point(delta_x / delta, delta_y / delta)

class Polygon:
    """代表一個由頂點列表構成的凸包。"""
    def __init__(self, points=None):
        self.points = points if points else []
    
    def __getitem__(self, key):
        return self.points[key]

    @staticmethod
    def _orientation(p, q, r):
        """
        [修改] 放寬浮點數比較時的絕對公差，以處理計算誤差。
        """
        val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
        # 使用一個更合適的絕對公差來判斷是否共線
        if math.isclose(val, 0, abs_tol=1e-6): 
            return 0
        return 1 if val > 0 else -1

    @staticmethod
    def build_hull(points):
        """從點集建立凸包 (Graham Scan)。"""
        if len(points) <= 2:
            return Polygon(points)
        
        points.sort(key=lambda p: (p.x, p.y))
        
        upper, lower = [], []
        for p in points:
            while len(upper) >= 2 and Polygon._orientation(upper[-2], upper[-1], p) >= 0:
                upper.pop()
            upper.append(p)

        for p in reversed(points):
            while len(lower) >= 2 and Polygon._orientation(lower[-2], lower[-1], p) >= 0:
                lower.pop()
            lower.append(p)
            
        return Polygon(upper[:-1] + lower[:-1])
        
    @staticmethod
    def merge_convex_hulls(hull_a, hull_b):
        """找到兩個凸包之間的上下公切線並返回切點索引。"""
        a_pts, b_pts = hull_a.points, hull_b.points
        a_size, b_size = len(a_pts), len(b_pts)

        # 找到 A 最右邊的點索引和 B 最左邊的點索引
        idx_a = max(range(a_size), key=lambda i: a_pts[i].x)
        idx_b = min(range(b_size), key=lambda i: b_pts[i].x)

        # 尋找上公切線
        upper_a, upper_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            while Polygon._orientation(b_pts[upper_b], a_pts[upper_a], a_pts[(upper_a + 1) % a_size]) >= 0:
                upper_a = (upper_a + 1) % a_size
            while Polygon._orientation(a_pts[upper_a], b_pts[upper_b], b_pts[(upper_b - 1 + b_size) % b_size]) <= 0:
                upper_b = (upper_b - 1 + b_size) % b_size
                done = False

        # 尋找下公切線
        lower_a, lower_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            while Polygon._orientation(b_pts[lower_b], a_pts[lower_a], a_pts[(lower_a - 1 + a_size) % a_size]) <= 0:
                lower_a = (lower_a - 1 + a_size) % a_size
            while Polygon._orientation(a_pts[lower_a], b_pts[lower_b], b_pts[(lower_b + 1) % b_size]) >= 0:
                lower_b = (lower_b + 1) % b_size
                done = False

        return upper_a, upper_b, lower_a, lower_b

# --- 2. VORONOI 圖演算法類別 ---

class VoronoiDiagram:
    def __init__(self, points, width, height):
        self.width = width
        self.height = height
        unique_points = sorted(list(set((p.x, p.y) for p in points)))
        self.points = [Point(x, y) for x, y in unique_points]
        self.voronoi_edges = []
        self.hyperplane_edges = []
        self.convex_hull = Polygon()

    def run(self):
        if len(self.points) < 2: return
        edges, hull, h_planes = self._divide_and_conquer(self.points)
        self.voronoi_edges = self._clip_edges(edges)
        # 將超平面從總邊中分離出來，以不同顏色繪製
        hyperplane_set = { (e.start, e.end) for e in self._clip_edges(h_planes) }
        final_edges = []
        final_h_planes = []
        for edge in self.voronoi_edges:
            if (edge.start, edge.end) in hyperplane_set or (edge.end, edge.start) in hyperplane_set:
                 final_h_planes.append(edge)
            else:
                 final_edges.append(edge)
        
        self.voronoi_edges = final_edges
        self.hyperplane_edges = final_h_planes
        self.convex_hull = hull
    def _are_points_collinear(self, points):
        """
        [修改] 改用頭尾兩點建立基準線，以提高數值計算的穩定性。
        """
        if len(points) < 3:
            return True
        
        # 使用點集中最遠的兩點（頭尾，因為點已排序）來定義基準線，這比使用相鄰兩點更穩定。
        p_start = points[0]
        p_end = points[-1]

        # 處理一種極端情況：如果所有輸入點都重合
        if p_start == p_end:
            return True

        # 檢查所有中間點是否都在這條更穩定的基準線上
        for i in range(1, len(points) - 1):
            if Polygon._orientation(p_start, p_end, points[i]) != 0:
                return False
            
        return True
    def _voronoi_collinear(self, points):
        """
        [新增] 為已排序的共線點集生成 Voronoi 圖。
        其結果為一系列平行的垂直平分線。
        """
        edges = []
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i+1]
            bisector = Line(p1, p2).get_perpendicular_bisector()
            edges.append(bisector)
        
        # 共線點的凸包就是連接最外側兩點的線段
        hull = Polygon([points[0], points[-1]])
        return edges, hull, edges
    
    def _divide_and_conquer(self, points):
        """
        [最終修正] 統一處理所有基礎案例 (n=2, n=3) 的返回值。
        """
        n = len(points)
        if n <= 1:
            return [], Polygon.build_hull(points), []

        if self._are_points_collinear(points):
            return self._voronoi_collinear(points)

        # [修正] 直接返回基礎案例函式的結果，不再手動解包和重新打包。
        # 因為 _voronoi2 和 _voronoi3 現已修正為返回標準的三個值。
        if n == 2:
            return self._voronoi2(points)

        if n == 3:
            return self._voronoi3(points)

        # 以下遞迴與合併部分保持不變
        mid = n // 2
        left_points, right_points = points[:mid], points[mid:]
        
        left_edges, left_hull, left_hps = self._divide_and_conquer(left_points)
        right_edges, right_hull, right_hps = self._divide_and_conquer(right_points)
        
        merged_edges, new_h_plane = self._merge((left_edges, left_hull), (right_edges, right_hull))
        
        final_hull = Polygon.build_hull(points)
        all_h_planes = left_hps + right_hps + new_h_plane
        
        return merged_edges, final_hull, all_h_planes
    
    def _voronoi2(self, points):
        """
        確保此函式返回三個值 (edges, hull, h_planes) 以保持一致性。
        """
        p1, p2 = points[0], points[1]
        bisector = Line(p1, p2).get_perpendicular_bisector()
        hull = Polygon.build_hull(points)
        # 對於基礎案例，邊緣和超平面是相同的
        return [bisector], hull, [bisector]
    def _voronoi3(self, points):
        
        p1, p2, p3 = points[0], points[1], points[2]
        hull = Polygon.build_hull(points)
        edges = []

        # --- 穩健性檢查 (保持不變) ---
        val = (p1.x - p2.x) * (p2.y - p3.y) - (p1.y - p2.y) * (p2.x - p3.x)
        
        if abs(val) < 1e-9:
            print("幾乎共線")
            # --- [關鍵修正] 處理共線/幾乎共線情況的邏輯 ---
            # 1. 從凸包中獲取三角形的三條邊
            hull_edges = []
            if len(hull.points) == 3:
                # Line 物件可以直接呼叫 get_perpendicular_bisector
                hull_edges.append(Line(hull.points[0], hull.points[1]))
                hull_edges.append(Line(hull.points[1], hull.points[2]))
                hull_edges.append(Line(hull.points[2], hull.points[0]))
            else:
                # 備用方案: 如果點過於退化無法形成三人凸包，則使用原始點
                hull_edges.append(Line(p1, p2)); hull_edges.append(Line(p2, p3))

            # 2. 找到最長邊的索引
            if hull_edges:
                edge_lengths_sq = [e.slope.length_sq() for e in hull_edges]
                max_len_idx = edge_lengths_sq.index(max(edge_lengths_sq))

                # 3. 只計算並加入兩條 "短邊" 的中垂線
                for i, edge in enumerate(hull_edges):
                    if i != max_len_idx:
                        print(i)
                        edges.append(edge.get_perpendicular_bisector())
        else:
            # --- 健康三角形的處理邏輯 (保持不變) ---
            b12 = Line(p1, p2).get_perpendicular_bisector()
            b23 = Line(p2, p3).get_perpendicular_bisector()
            circumcenter = Line.intersection(b12, b23)
            
            if circumcenter:
                dir12 = (p2 - p1).normal()
                dir23 = (p3 - p2).normal()
                dir31 = (p1 - p3).normal()
                multiplier = (self.width + self.height) * 2

                edges.append(Line(circumcenter, circumcenter + (dir12 * multiplier), p1, p2))
                edges.append(Line(circumcenter, circumcenter + (dir23 * multiplier), p2, p3))
                edges.append(Line(circumcenter, circumcenter + (dir31 * multiplier), p3, p1))
            else:
                edges.append(b12); edges.append(b23)
        
        return edges, hull, edges

    def _merge(self, left_data, right_data):
        left_edges, left_hull = left_data
        right_edges, right_hull = right_data
        
        # 1. 合併凸包並找到公切線
        ua_idx, ub_idx, la_idx, lb_idx = Polygon.merge_convex_hulls(left_hull, right_hull)
        
        p_ua, p_ub = left_hull[ua_idx], right_hull[ub_idx]
        p_la, p_lb = left_hull[la_idx], right_hull[lb_idx]
        
        # 2. 追蹤超平面
        hyperplane = []
        scan_trends = []
        scan_line = Line(p_ua, p_ub)
        lower_tangent = Line(p_la, p_lb)

        # 複製邊列表，以防修改
        l_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in left_edges]
        r_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in right_edges]
        
        while True:
            # 2.1 計算平分線並調整方向
            scan_pb = scan_line.get_perpendicular_bisector()
            scan_pb.to_direction('d') # C++ 版邏輯是朝下
            
            # 2.2 連接上一段超平面
            if hyperplane:
                scan_pb.start = hyperplane[-1].end

            # 2.3 尋找最近的交點
            l_is_pt, l_is_idx = self._find_near_intra_is(scan_pb, l_edges)
            r_is_pt, r_is_idx = self._find_near_intra_is(scan_pb, r_edges)
            
            # 2.4 決定下一個事件點
            near_is_pt, choose_dir, near_is_idx = self._find_near_inter_is(
                scan_pb, l_is_pt, r_is_pt, l_is_idx, r_is_idx
            )
            
            scan_trends.append(Trend(direction=choose_dir, index=near_is_idx))
            
            # 2.5 添加超平面線段
            is_lower = (scan_line.norm_start == lower_tangent.norm_start and scan_line.norm_end == lower_tangent.norm_end)
            
            if near_is_pt and not is_lower:
                scan_pb.end = near_is_pt
            else:
                 # 如果沒有交點或已到達下切線，則延伸到邊界
                 clipped = self._cohen_sutherland_clip(scan_pb.start, scan_pb.end)
                 if clipped:
                     scan_pb.start, scan_pb.end = clipped

            hyperplane.append(scan_pb)

            # 2.6 檢查終止條件
            if is_lower: break
            if not near_is_pt: break # 無法再前進

            # 2.7 移動到下一個掃描線
            scan_line = self._move_scan_next_loop(scan_line, l_edges, r_edges, l_is_idx, r_is_idx, choose_dir)

        # 3. 更新轉向趨勢
        self._update_trend(hyperplane, scan_trends)
        
        # 4. 消除與修剪
        self._elimination(hyperplane, scan_trends, l_edges, r_edges)
        
        # 5. 最終合併
        final_edges = l_edges + r_edges + hyperplane
        return final_edges, hyperplane

    def _find_near_intra_is(self, scan_pb, edges):
        """在單個子圖中尋找最近的交點。"""
        nearest_is_pt = None
        nearest_is_idx = -1
        min_dist_sq = float('inf')

        for i, edge in enumerate(edges):
            # 檢查邊是否與 scan_pb 的站點相關
            if not (scan_pb.norm_start == edge.norm_start or \
                    scan_pb.norm_start == edge.norm_end or \
                    scan_pb.norm_end == edge.norm_start or \
                    scan_pb.norm_end == edge.norm_end):
                continue

            is_pt = Line.intersection(scan_pb, edge)
            if is_pt and is_pt != scan_pb.start:
                dist_sq = (is_pt - scan_pb.start).length_sq()
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    nearest_is_pt = is_pt
                    nearest_is_idx = i
        
        return nearest_is_pt, nearest_is_idx
    
    def _find_near_inter_is(self, scan_pb, l_pt, r_pt, l_idx, r_idx):
        """比較左右兩側的交點，找出更近的一個。"""
        if l_pt and r_pt:
            dist_l_sq = (l_pt - scan_pb.start).length_sq()
            dist_r_sq = (r_pt - scan_pb.start).length_sq()
            if dist_l_sq < dist_r_sq:
                return l_pt, 1, l_idx # 1: 左側
            else:
                return r_pt, 2, r_idx # 2: 右側
        elif l_pt:
            return l_pt, 1, l_idx
        elif r_pt:
            return r_pt, 2, r_idx
        else:
            return None, 0, -1 # 0: 無

    def _move_scan_next_loop(self, scan_line, l_edges, r_edges, l_idx, r_idx, choose):
        new_start, new_end = scan_line.norm_start, scan_line.norm_end
        if choose == 1: # 左側事件
            edge = l_edges[l_idx]
            if new_start == edge.norm_start:
                new_start = edge.norm_end
            elif new_start == edge.norm_end:
                new_start = edge.norm_start
        elif choose == 2: # 右側事件
            edge = r_edges[r_idx]
            if new_end == edge.norm_start:
                new_end = edge.norm_end
            elif new_end == edge.norm_end:
                new_end = edge.norm_start
        return Line(new_start, new_end)

    def _update_trend(self, hyperplane, scan_trends):
        """計算超平面每個轉角的轉向。"""
        for i in range(len(hyperplane) - 1):
            cp = Point.cross_product(hyperplane[i].slope, hyperplane[i+1].slope)
            if cp > 1e-9: # 逆時針
                scan_trends[i].trend = 1
            elif cp < -1e-9: # 順時針
                scan_trends[i].trend = 2

    def _elimination(self, hyperplane, scan_trends, l_edges, r_edges):
        """根據追蹤記錄修剪舊的 Voronoi 邊。"""
        for i in range(len(scan_trends)):
            trend_info = scan_trends[i]
            if trend_info.direction == 0 or trend_info.index == -1: continue

            edges = l_edges if trend_info.direction == 1 else r_edges
            edge_to_prune = edges[trend_info.index]
            intersection_pt = hyperplane[i].end

            # 如果交點在邊界上，則不進行修剪
            if intersection_pt.is_on_boundary(self.width, self.height): continue

            # 根據趨勢決定修剪哪一端
            # 這是一個簡化邏輯，完整邏輯需要處理更多邊界情況
            dist_start = (edge_to_prune.start - intersection_pt).length_sq()
            dist_end = (edge_to_prune.end - intersection_pt).length_sq()

            if trend_info.trend == 1: # 逆時針，通常修剪較遠的一端
                if dist_start > dist_end:
                    edge_to_prune.start = intersection_pt
                else:
                    edge_to_prune.end = intersection_pt
            elif trend_info.trend == 2: # 順時針，通常修剪較近的一端
                 if dist_start < dist_end:
                    edge_to_prune.start = intersection_pt
                 else:
                    edge_to_prune.end = intersection_pt

    def _clip_edges(self, edges):
        clipped = []
        for edge in edges:
            clipped_edge = self._cohen_sutherland_clip(edge.start, edge.end)
            if clipped_edge:
                p1, p2 = clipped_edge
                if p1 != p2:
                    # 傳遞 norm 屬性
                    new_edge = Edge(p1, p2)
                    if isinstance(edge, Line):
                       new_edge.norm_start = edge.norm_start
                       new_edge.norm_end = edge.norm_end
                    clipped.append(new_edge)
        return clipped
    
    def _cohen_sutherland_clip(self, p1, p2):
        """Cohen-Sutherland 線段裁剪演算法。"""
        INSIDE, LEFT, RIGHT, BOTTOM, TOP = 0, 1, 2, 4, 8
        
        def _get_code(p):
            code = INSIDE
            if p.x < 0: code |= LEFT
            elif p.x > self.width: code |= RIGHT
            if p.y < 0: code |= BOTTOM
            elif p.y > self.height: code |= TOP
            return code

        code1, code2 = _get_code(p1), _get_code(p2)
        accept = False

        while True:
            if not (code1 | code2): # 兩個點都在內部
                accept = True
                break
            elif code1 & code2: # 兩個點都在同一個外部區域
                break
            else:
                code_out = code1 if code1 else code2
                x, y = 0.0, 0.0

                if code_out & TOP:
                    x = p1.x + (p2.x - p1.x) * (self.height - p1.y) / (p2.y - p1.y) if p2.y != p1.y else p1.x
                    y = self.height
                elif code_out & BOTTOM:
                    x = p1.x + (p2.x - p1.x) * (0 - p1.y) / (p2.y - p1.y) if p2.y != p1.y else p1.x
                    y = 0
                elif code_out & RIGHT:
                    y = p1.y + (p2.y - p1.y) * (self.width - p1.x) / (p2.x - p1.x) if p2.x != p1.x else p1.y
                    x = self.width
                elif code_out & LEFT:
                    y = p1.y + (p2.y - p1.y) * (0 - p1.x) / (p2.x - p1.x) if p2.x != p1.x else p1.y
                    x = 0

                if code_out == code1:
                    p1 = Point(x, y)
                    code1 = _get_code(p1)
                else:
                    p2 = Point(x, y)
                    code2 = _get_code(p2)
        
        return (p1, p2) if accept else None


# --- 3. 應用程式 GUI 類別 ---

class Application(tk.Tk):
    """使用 tkinter 的主 GUI 應用程式類別。"""
    def __init__(self, width=600, height=600):
        super().__init__()
        self.width = width
        self.height = height
        
        self.title("Voronoi 圖產生器")
        self.geometry(f"{width + 220}x{height + 50}")
        self.resizable(False, False)

        self.points = []
        self.voronoi_edges = []
        self.hyperplane_edges = [] # 用於顯示超平面的佔位符
        self.convex_hull = None

        # --- 控制面板 ---
        control_frame = ttk.Frame(self, padding=10)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Label(control_frame, text="控制項", font=("Arial", 14, "bold")).pack(pady=10)

        run_button = ttk.Button(control_frame, text="執行 Voronoi", command=self.run_voronoi)
        run_button.pack(fill=tk.X, pady=5)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        ttk.Label(control_frame, text="隨機產生點數：").pack()
        self.random_points_entry = ttk.Entry(control_frame)
        self.random_points_entry.pack(fill=tk.X)
        self.random_points_entry.insert(0, "20")
        
        random_button = ttk.Button(control_frame, text="產生", command=self.generate_random_points)
        random_button.pack(fill=tk.X, pady=5)

        ttk.Separator(control_frame, orient='horizontal').pack(fill='x', pady=10)

        clear_button = ttk.Button(control_frame, text="清除畫布", command=self.clear_canvas)
        clear_button.pack(fill=tk.X, pady=5)

        # --- 繪圖畫布 (Canvas) ---
        self.canvas = tk.Canvas(self, width=self.width, height=self.height, bg="white", highlightthickness=1, highlightbackground="black")
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self.add_point_on_click)

    def add_point_on_click(self, event):
        """當使用者點擊畫布時，添加一個點。"""
        self.points.append(Point(event.x, event.y))
        self.draw()

    def generate_random_points(self):
        """產生指定數量的隨機點。"""
        try:
            num_points = int(self.random_points_entry.get())
            if num_points <= 0: raise ValueError
        except (ValueError, TypeError):
            messagebox.showerror("輸入無效", "請輸入一個正整數。")
            return

        self.clear_canvas()
        for _ in range(num_points):
            x = random.randint(10, self.width - 10)
            y = random.randint(10, self.height - 10)
            self.points.append(Point(x, y))
        self.draw()

    def run_voronoi(self):
        """
        [修改] 增加對共線情況的判斷，確保在任何情況下都能印出有意義的頂點座標。
        """
        if len(self.points) < 2:
            messagebox.showinfo("點數不足", "至少需要 2 個點才能計算 Voronoi 圖。")
            return
            
        voronoi = VoronoiDiagram(self.points, self.width, self.height)
        voronoi.run()
        
        self.voronoi_edges = voronoi.voronoi_edges
        self.hyperplane_edges = voronoi.hyperplane_edges
        self.convex_hull = voronoi.convex_hull

        # --- [修改] 在控制台印出 Voronoi 頂點座標 (增加共線情況處理) ---
        print("\n" + "="*28)
        print("Voronoi 頂點座標")
        for i in self.points:
            print(i) 
        print("="*28)

        # 判斷當前計算的點集是否為共線情況
        # voronoi.points 是經過排序和去重的點集，最適合用來判斷
        is_collinear_case = voronoi._are_points_collinear(voronoi.points)

        all_edges = self.voronoi_edges + self.hyperplane_edges
        vertices_to_print = set()

        if is_collinear_case:
            print("(共線情況：顯示邊於畫布邊界的端點)")
            # 對於共線情況，收集所有邊的端點
            for edge in all_edges:
                vertices_to_print.add(edge.start)
                vertices_to_print.add(edge.end)
        else:
            print("(一般情況：顯示內部交點)")
            # 對於一般情況，只收集不在邊界上的內部頂點
            for edge in all_edges:
                if not edge.start.is_on_boundary(self.width, self.height):
                    vertices_to_print.add(edge.start)
                if not edge.end.is_on_boundary(self.width, self.height):
                    vertices_to_print.add(edge.end)

        if not vertices_to_print:
            print("(無可顯示之頂點)")
        else:
            # 為了讓輸出順序固定，可以先轉換為列表並按 y, x 排序
            sorted_vertices = sorted(list(vertices_to_print), key=lambda p: (p.y, p.x))
            for i, vertex in enumerate(sorted_vertices):
                label = "邊界端點" if is_collinear_case else "內部頂點"
                print(f"  {label} {i+1}: ({vertex.x:.2f}, {vertex.y:.2f})")
                
        print("="*28 + "\n")
        # --- 修改結束 ---

        self.draw()

    def clear_canvas(self):
        """清除畫布和所有資料。"""
        self.points = []
        self.voronoi_edges = []
        self.hyperplane_edges = []
        self.convex_hull = None
        self.canvas.delete("all")

    def draw(self):
        """
        [修改] 繪圖函式中，增加對線段狀凸包的處理。
        """
        self.canvas.delete("all")
        
        # 繪製 Voronoi 邊 (黑色)
        if self.voronoi_edges:
            for edge in self.voronoi_edges:
                self.canvas.create_line(edge.start.x, edge.start.y, edge.end.x, edge.end.y, fill="black", width=1.5)
        
        # 繪製超平面 (藍色)
        if self.hyperplane_edges:
            for edge in self.hyperplane_edges:
                self.canvas.create_line(edge.start.x, edge.start.y, edge.end.x, edge.end.y, fill="blue", width=1.5)

        # 繪製凸包 (綠色)
        if self.convex_hull and len(self.convex_hull.points) > 1:
            points_flat = [coord for p in self.convex_hull.points for coord in (p.x, p.y)]
            
            # [新增的邏輯] 如果凸包只有兩個點 (即一條線)，則使用 create_line 繪製
            if len(self.convex_hull.points) == 2:
                self.canvas.create_line(points_flat, fill="green", width=2)
            else:
                self.canvas.create_polygon(points_flat, outline="green", fill="", width=2)

        # 繪製站點 (紅色)
        for p in self.points:
            self.canvas.create_oval(p.x - 3, p.y - 3, p.x + 3, p.y + 3, fill="red", outline="black")

if __name__ == "__main__":
    # 增加 Point 類的輔助方法
    Point.normal = lambda self: Point(-self.y, self.x)
    app = Application()
    app.mainloop()