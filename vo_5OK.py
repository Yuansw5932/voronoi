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
        self.used = False
        try:
            if hasattr((p2 - p1), 'normal'):
                normal_vec = (p2 - p1).normal()
                self.a, self.b = normal_vec.x, normal_vec.y
                self.c = -1 * (self.a * self.start.x + self.b * self.start.y)
            else:
                 self.a, self.b, self.c = 0,0,0
        except AttributeError:
            self.a, self.b, self.c = 0, 0, 0
    def __repr__(self): return f"Line(S={self.start}, E={self.end}, Normals=[{self.norm_start}, {self.norm_end}])"
    def to_direction(self, direction='d'):
        s = self.slope
        if direction == 'd' and s.y > 0: self.start, self.end = self.end, self.start
        elif direction == 'u' and s.y < 0: self.start, self.end = self.end, self.start
        elif direction == 'r' and s.x < 0: self.start, self.end = self.end, self.start
        elif direction == 'l' and s.x > 0: self.start, self.end = self.end, self.start
            
    def get_perpendicular_bisector(self):
        mid = (self.norm_start + self.norm_end) / 2
        normal = Point(-(self.norm_end.y - self.norm_start.y), self.norm_end.x - self.norm_start.x)
        return Line(mid + normal * 2000, mid - normal * 2000, self.norm_start, self.norm_end)
    @staticmethod
    def intersection(l1, l2):
        if not all(hasattr(l, 'a') for l in [l1, l2]): return None
        delta = l1.a * l2.b - l2.a * l1.b
        if math.isclose(delta, 0): return None
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
        sorted_points = sorted(points, key=lambda p: (p.x, p.y))
        upper, lower = [], []
        for p in sorted_points:
            while len(upper) >= 2 and Polygon._orientation(upper[-2], upper[-1], p) >= 0: upper.pop()
            upper.append(p)
        for p in reversed(sorted_points):
            while len(lower) >= 2 and Polygon._orientation(lower[-2], lower[-1], p) >= 0: lower.pop()
            lower.append(p)
        return Polygon(upper[:-1] + lower[:-1])
    @staticmethod
    def merge_convex_hulls(hull_a, hull_b):
        a_pts, b_pts = hull_a.points, hull_b.points
        a_size, b_size = len(a_pts), len(b_pts)
        if not a_pts or not b_pts: return 0,0,0,0
        idx_a, idx_b = max(range(a_size), key=lambda i: a_pts[i].x), min(range(b_size), key=lambda i: b_pts[i].x)
        upper_a, upper_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            while Polygon._orientation(b_pts[upper_b], a_pts[upper_a], a_pts[(upper_a + 1) % a_size]) >= 0:
                upper_a = (upper_a + 1) % a_size
            while Polygon._orientation(a_pts[upper_a], b_pts[upper_b], b_pts[(b_size + upper_b - 1) % b_size]) <= 0:
                upper_b = (b_size + upper_b - 1) % b_size
                done = False
        lower_a, lower_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            while Polygon._orientation(a_pts[lower_a], b_pts[lower_b], b_pts[(lower_b + 1) % b_size]) >= 0:
                lower_b = (lower_b + 1) % b_size
            while Polygon._orientation(b_pts[lower_b], a_pts[lower_a], a_pts[(a_size + lower_a - 1) % a_size]) <= 0:
                lower_a = (a_size + lower_a - 1) % a_size
                done = False
        return upper_a, upper_b, lower_a, lower_b

# --- 2. VORONOI 圖演算法類別 ---

class VoronoiDiagram:
    def __init__(self, points, width, height):
        self.width, self.height = width, height
        unique_points_tuples = sorted(list(set((p.x, p.y) for p in points)))
        self.points = [Point(x, y) for x, y in unique_points_tuples]
        self.steps = []

    def run(self, step_mode=False):
        self.steps.clear()
        if len(self.points) < 2: return [], Polygon(), []
        sorted_points = sorted(self.points, key=lambda p: (p.x, p.y))
        edges, hull, h_planes = self._divide_and_conquer(sorted_points, step_mode)
        
        if not step_mode:
            final_edges = self._clip_edges(edges)
            final_h_planes = self._clip_edges(h_planes) if h_planes else []
            return final_edges, hull, final_h_planes
        return None, None, None

    def _divide_and_conquer(self, points, step_mode):
        print(f"[DEBUG] D&C called with {len(points)} points: {[p for p in points]}")
        n = len(points)
        if n <= 1: 
            return [], Polygon.build_hull(points), []

        if n <= 3:
            edges, hull, h_planes = self._voronoi_base(points)
            if step_mode:
                clipped_edges = self._clip_edges(edges)
                self.steps.append(StepRecord(f"基礎案例: 計算 {n} 個點", points, left_edges=clipped_edges, convex_hull=hull))
            return edges, hull, h_planes

        mid = n // 2
        left_points, right_points = points[:mid], points[mid:]
        print(f"[DEBUG] Splitting into Left: {left_points} and Right: {right_points}")
        
        left_edges, left_hull, left_hps = self._divide_and_conquer(left_points, step_mode)
        right_edges, right_hull, right_hps = self._divide_and_conquer(right_points, step_mode)

        print(f"[DEBUG] Left recursion returned {len(left_edges)} edges.")
        print(f"[DEBUG] Right recursion returned {len(right_edges)} edges.")
        clipped_left_edges = self._clip_edges(left_edges)
        clipped_right_edges = self._clip_edges(right_edges)
        
        merged_edges, new_h_plane = self._merge(
            (clipped_left_edges, left_hull, left_points), 
            (clipped_right_edges, right_hull, right_points), 
            step_mode
        )
        
        final_hull = Polygon.build_hull(points)
        all_h_planes = (left_hps if left_hps else []) + (right_hps if right_hps else []) + (new_h_plane if new_h_plane else [])
        
        if step_mode:
            clipped_final_edges = self._clip_edges(merged_edges)
            self.steps.append(StepRecord(f"完成合併: {len(points)} 個點", points, left_edges=clipped_final_edges, convex_hull=final_hull))
        
        print(f"[DEBUG] D&C for {len(points)} points finished.")
        return merged_edges, final_hull, all_h_planes
        
    def _voronoi_base(self, points):
        if len(points) == 2:
            p1, p2 = points[0], points[1]
            bisector = Line(p1, p2).get_perpendicular_bisector()
            hull = Polygon.build_hull(points)
            return [bisector], hull, [bisector]
        
        p1, p2, p3 = points[0], points[1], points[2]
        hull = Polygon.build_hull(points)
        edges = []

        if abs(Point.cross_product(p2 - p1, p3 - p1)) < 1e-9:
            all_points = sorted([p1, p2, p3], key=lambda p: (p.x, p.y))
            edges.append(Line(all_points[0], all_points[1]).get_perpendicular_bisector())
            edges.append(Line(all_points[1], all_points[2]).get_perpendicular_bisector())
            return edges, hull, edges
        
        b12 = Line(p1, p2).get_perpendicular_bisector()
        b23 = Line(p2, p3).get_perpendicular_bisector()
        circumcenter = Line.intersection(b12, b23)

        if circumcenter:
            multiplier = (self.width + self.height) * 2
            dir12 = (p2 - p1).normal()
            ref_vec_12 = p1 - p3
            if (ref_vec_12.x * dir12.x + ref_vec_12.y * dir12.y) < 0: dir12 = dir12 * -1
            edges.append(Line(circumcenter, circumcenter + dir12 * multiplier, p1, p2))

            dir23 = (p3 - p2).normal()
            ref_vec_23 = p2 - p1
            if (ref_vec_23.x * dir23.x + ref_vec_23.y * dir23.y) < 0: dir23 = dir23 * -1
            edges.append(Line(circumcenter, circumcenter + dir23 * multiplier, p2, p3))

            dir31 = (p1 - p3).normal()
            ref_vec_31 = p3 - p2
            if (ref_vec_31.x * dir31.x + ref_vec_31.y * dir31.y) < 0: dir31 = dir31 * -1
            edges.append(Line(circumcenter, circumcenter + dir31 * multiplier, p3, p1))
        else:
            edges.append(b12)
            edges.append(b23)
        return edges, hull, edges

    def _merge(self, left_data, right_data, step_mode):
        print("\n[DEBUG] --- MERGE ---")
        left_edges, left_hull, left_points = left_data
        right_edges, right_hull, right_points = right_data
        print(f"[DEBUG] Merging {len(left_points)} left points and {len(right_points)} right points.")
        
        ua_idx, ub_idx, la_idx, lb_idx = Polygon.merge_convex_hulls(left_hull, right_hull)
        if not left_hull.points or not right_hull.points: return left_edges + right_edges, []
        
        p_ua, p_ub = left_hull[ua_idx], right_hull[ub_idx]
        p_la, p_lb = left_hull[la_idx], right_hull[lb_idx]
        print(f"[DEBUG] Upper tangent: {p_ua} to {p_ub}")
        print(f"[DEBUG] Lower tangent: {p_la} to {p_lb}")

        if step_mode:
            merged_hull = Polygon.build_hull(left_points + right_points)
            self.steps.append(StepRecord("步驟 1/4: 合併凸包", left_points + right_points, left_edges, right_edges, convex_hull=merged_hull))

        hyperplane, scan_trends = [], []
        scan_line = Line(p_ua, p_ub)
        lower_tangent = Line(p_la, p_lb)
        
        l_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in left_edges]
        r_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in right_edges]
        
        last_v_pt = None
        loop_count = 0
        while True:
            loop_count += 1
            if loop_count > (len(left_points) + len(right_points)) * 4:
                print("[DEBUG] *** SAFETY BREAK: Infinite loop detected in _merge ***")
                break
            
            print(f"\n[DEBUG] Hyperplane Loop #{loop_count}: Scan line from {scan_line.norm_start} to {scan_line.norm_end}")
            bisector = scan_line.get_perpendicular_bisector()

            l_is_pt, l_is_idx = self._find_near_intra_is(bisector, l_edges)
            r_is_pt, r_is_idx = self._find_near_intra_is(bisector, r_edges)
            near_is_pt, choose_dir, near_is_idx = self._find_near_inter_is(bisector, l_is_pt, r_is_pt, l_is_idx, r_is_idx)
            print(f"[DEBUG]   Nearest intersection found: point={near_is_pt}, from side={choose_dir}, edge_index={near_is_idx}")
            
            start_pt = last_v_pt
            if start_pt is None:
                clipped_pts = self._cohen_sutherland_clip(bisector.start, bisector.end)
                if clipped_pts:
                    p1_clip, p2_clip = clipped_pts
                    start_pt = p1_clip if p1_clip.y > p2_clip.y else p2_clip
                    print(f"[DEBUG]   First segment. Start on boundary: {start_pt}")
                else:
                    print("[DEBUG]   ERROR: Initial bisector does not cross canvas.")
                    break
            
            is_lower = Point.is_similar(scan_line.norm_start, lower_tangent.norm_start) and Point.is_similar(scan_line.norm_end, lower_tangent.norm_end)

            end_pt = near_is_pt
            if end_pt is None or is_lower:
                bisector.to_direction('d')
                end_pt = start_pt + (bisector.end - bisector.start)
                print(f"[DEBUG]   No more intersections or lower tangent. Creating final ray to {end_pt}")
                last_v_pt = None
            else:
                last_v_pt = end_pt

            segment = Line(start_pt, end_pt, scan_line.norm_start, scan_line.norm_end)
            hyperplane.append(segment)
            print(f"[DEBUG]   Created segment: {segment}")
            scan_trends.append(Trend(direction=choose_dir, index=near_is_idx))

            if last_v_pt is None or (near_is_pt and is_lower):
                print("[DEBUG]   Loop termination condition met.")
                break

            if choose_dir == 1 and near_is_idx != -1:
                print(f"[DEBUG]   Marking left edge {near_is_idx} as used.")
                l_edges[near_is_idx].used = True
            elif choose_dir == 2 and near_is_idx != -1:
                print(f"[DEBUG]   Marking right edge {near_is_idx} as used.")
                r_edges[near_is_idx].used = True

            print(f"[DEBUG]   Updating scan line...")
            scan_line = self._move_scan_next_loop(scan_line, l_edges, r_edges, l_is_idx, r_is_idx, choose_dir)

        if step_mode: self.steps.append(StepRecord("步驟 2/4: 追蹤超平面", left_points + right_points, l_edges, r_edges, hyperplane))
        
        print("[DEBUG] --- Calling _update_trend and _elimination ---")
        self._update_trend(hyperplane, scan_trends)
        self._elimination(hyperplane, scan_trends, l_edges, r_edges)
        
        if step_mode: self.steps.append(StepRecord("步驟 3/4: 執行消線", left_points + right_points, l_edges, r_edges, hyperplane))

        final_edges = l_edges + r_edges + hyperplane
        print("[DEBUG] --- MERGE FINISHED ---")
        return final_edges, hyperplane
    
    
    def _find_near_intra_is(self, scan_pb, edges):
        nearest_is_pt, nearest_is_idx, max_y = None, -1, -float('inf')
        scan_slope = scan_pb.slope
        for i, edge in enumerate(edges):
            print(f"[DEBUG]     _find_near_intra_is: Checking edge {i}: {edge}")
            if hasattr(edge, 'used') and edge.used:
                print(f"[DEBUG]       Edge {i} is already used. Skipping.")
                continue
            if not hasattr(edge, 'norm_start') or not hasattr(edge, 'norm_end'): continue
            if not (scan_pb.norm_start == edge.norm_start or scan_pb.norm_start == edge.norm_end or \
                    scan_pb.norm_end == edge.norm_start or scan_pb.norm_end == edge.norm_end):
                continue
            
            print(f"[DEBUG]       Edge has common point. Checking intersection.")
            is_pt = Line.intersection(scan_pb, edge)
            if is_pt and not Point.is_similar(is_pt, scan_pb.start, tol=1e-3):
                vec_to_is = is_pt - scan_pb.start
                if (vec_to_is.x * scan_slope.x + vec_to_is.y * scan_slope.y) > 1e-9:
                    if is_pt.y > max_y:
                        print(f"[DEBUG]       New highest-Y intersection found: {is_pt}")
                        max_y, nearest_is_pt, nearest_is_idx = is_pt.y, is_pt, i
        return nearest_is_pt, nearest_is_idx

    def _find_near_inter_is(self, scan_pb, l_pt, r_pt, l_idx, r_idx):
        if l_pt and r_pt:
            if math.isclose(l_pt.y, r_pt.y, abs_tol=1e-3):
                dist_l_sq = (l_pt - scan_pb.start).length_sq()
                dist_r_sq = (r_pt - scan_pb.start).length_sq()
                return (l_pt, 1, l_idx) if dist_l_sq < dist_r_sq else (r_pt, 2, r_idx)
            return (l_pt, 1, l_idx) if l_pt.y > r_pt.y else (r_pt, 2, r_idx)
        return (l_pt, 1, l_idx) if l_pt else (r_pt, 2, r_idx) if r_pt else (None, 0, -1)

    def _move_scan_next_loop(self, scan_line, l_edges, r_edges, l_idx, r_idx, choose):
        new_start, new_end = scan_line.norm_start, scan_line.norm_end
        edge_to_check = None
        if choose == 1 and l_idx != -1:
            edge_to_check = l_edges[l_idx]
            print(f"[DEBUG]     _move_scan_next_loop: Using left edge: {edge_to_check}")
            new_start = edge_to_check.norm_end if new_start == edge_to_check.norm_start else edge_to_check.norm_start
        elif choose == 2 and r_idx != -1:
            edge_to_check = r_edges[r_idx]
            print(f"[DEBUG]     _move_scan_next_loop: Using right edge: {edge_to_check}")
            new_end = edge_to_check.norm_end if new_end == edge_to_check.norm_start else edge_to_check.norm_start
        
        return Line(new_start, new_end)

    def _update_trend(self, hyperplane, scan_trends):
        for i in range(len(hyperplane) - 1):
            cp = Point.cross_product(hyperplane[i].slope, hyperplane[i+1].slope)
            if cp > 1e-9: scan_trends[i].trend = 2
            elif cp < -1e-9: scan_trends[i].trend = 1
    
    def _elimination(self, hyperplane, scan_trends, l_edges, r_edges):
        print("[DEBUG]   _elimination starts (Final Logic based on Trend)...")
        # A turn happens at the end of segment i, involving segment i+1
        for i in range(len(hyperplane) - 1):
            turn_info = scan_trends[i] # Trend info FOR this turn
            edge_info = scan_trends[i] # The edge that was hit to CAUSE this turn

            if edge_info.direction == 0:
                continue

            intersection_pt = hyperplane[i].end
            if intersection_pt is None or intersection_pt.is_on_boundary(self.width, self.height):
                continue

            edges_list = l_edges if edge_info.direction == 1 else r_edges
            if not (0 <= edge_info.index < len(edges_list)):
                continue
            
            edge_to_prune = edges_list[edge_info.index]
            print(f"[DEBUG]     Turn {i}: Trend is {turn_info.trend}. Pruning edge {edge_info.index} from side {edge_info.direction} at {intersection_pt}")

            # The vector of the hyperplane segment leading into the intersection
            v_hyper = hyperplane[i].slope

            # Candidate vector if we keep the 'end' point
            v_new_if_keep_end = edge_to_prune.end - intersection_pt
            cross_prod_end = Point.cross_product(v_hyper, v_new_if_keep_end)

            # Candidate vector if we keep the 'start' point
            v_new_if_keep_start = edge_to_prune.start - intersection_pt
            cross_prod_start = Point.cross_product(v_hyper, v_new_if_keep_start)

            if turn_info.trend == 1: # Trend LEFT (Counter-Clockwise) -> We need the positive cross product
                print(f"[DEBUG]       Turn is LEFT. Cross products are: (keep_end={cross_prod_end:.2f}, keep_start={cross_prod_start:.2f})")
                # Choose the endpoint that results in a positive cross product
                if cross_prod_end > cross_prod_start:
                    print(f"[DEBUG]       Keeping END point. Pruning START from {edge_to_prune.start}")
                    edge_to_prune.start = intersection_pt
                else:
                    print(f"[DEBUG]       Keeping START point. Pruning END from {edge_to_prune.end}")
                    edge_to_prune.end = intersection_pt
            elif turn_info.trend == 2: # Trend RIGHT (Clockwise) -> We need the negative cross product
                print(f"[DEBUG]       Turn is RIGHT. Cross products are: (keep_end={cross_prod_end:.2f}, keep_start={cross_prod_start:.2f})")
                # Choose the endpoint that results in a negative cross product
                if cross_prod_end < cross_prod_start:
                    print(f"[DEBUG]       Keeping END point. Pruning START from {edge_to_prune.start}")
                    edge_to_prune.start = intersection_pt
                else:
                    print(f"[DEBUG]       Keeping START point. Pruning END from {edge_to_prune.end}")
                    edge_to_prune.end = intersection_pt
        print("[DEBUG]   _elimination finished.")

    def _clip_edges(self, edges):
        if not edges: return []
        clipped = []
        for edge in edges:
            if not isinstance(edge, Edge): continue
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
            
            if code_out & TOP:
                x = p1.x + dx * (self.height - p1.y) / dy if dy != 0 else p1.x
                y = self.height
            elif code_out & BOTTOM:
                x = p1.x + dx * (0 - p1.y) / dy if dy != 0 else p1.x
                y = 0
            elif code_out & RIGHT:
                y = p1.y + dy * (self.width - p1.x) / dx if dx != 0 else p1.y
                x = self.width
            elif code_out & LEFT:
                y = p1.y + dy * (0 - p1.x) / dx if dx != 0 else p1.y
                x = 0
            
            if code_out == code1:
                p1, code1 = Point(x, y), _get_code(Point(x, y))
            else:
                p2, code2 = Point(x, y), _get_code(Point(x, y))

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

        point_frame = ttk.LabelFrame(control_frame, text="1. 新增站點", padding=10)
        point_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(point_frame, text="隨機產生點數：").pack()
        self.random_points_entry = ttk.Entry(point_frame)
        self.random_points_entry.pack(fill=tk.X, pady=(0, 5))
        self.random_points_entry.insert(0, "6") 
        ttk.Button(point_frame, text="隨機產生", command=self.generate_random_points).pack(fill=tk.X)
        ttk.Button(point_frame, text="清除畫布", command=self.clear_canvas).pack(fill=tk.X, pady=(5,0))

        run_frame = ttk.LabelFrame(control_frame, text="2. 執行演算法", padding=10)
        run_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(run_frame, text="執行 (一次完成)", command=self.run_normal).pack(fill=tk.X)
        ttk.Button(run_frame, text="產生步驟", command=self.run_step_by_step_generate).pack(fill=tk.X, pady=5)
        
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
        self.points.append(Point(event.x, self.height - event.y))
        self.reset_steps()
        self.draw_points()

    def generate_random_points(self):
        self.clear_canvas()
        try:
            num = int(self.random_points_entry.get())
            for _ in range(num): self.points.append(Point(random.randint(10, self.width - 10), random.randint(10, self.height - 10)))
        except (ValueError, TypeError): messagebox.showerror("無效輸入", "請輸入一個有效的整數。")
        self.draw_points()

    def run_normal(self):
        if len(self.points) < 2: return
        self.reset_steps()
        voronoi = VoronoiDiagram(self.points, self.width, self.height)
        edges, hull, h_planes = voronoi.run(step_mode=False)
        if edges is None:
            messagebox.showerror("執行錯誤", "演算法執行過程中發生錯誤，可能進入了無限迴圈或遇到未處理的邊界情況。")
            return
        all_final_edges = edges
        self.draw_all(
            points=self.points,
            left_edges=all_final_edges,
            right_edges=[],
            hyperplane=[],
            convex_hull_pts=hull.points if hull else [],
            is_final_run=True
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
        if not self.voronoi_steps: return
        step_record = self.voronoi_steps[self.current_step]
        self.step_label.config(text=f"第 {self.current_step + 1}/{len(self.voronoi_steps)} 步\n{step_record.description}")
        self.draw_all(
            points=step_record.points, 
            left_edges=step_record.left_edges, 
            right_edges=step_record.right_edges, 
            hyperplane=step_record.hyperplane, 
            convex_hull_pts=step_record.convex_hull,
            is_final_run=False
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
            y = self.height - p.y
            self.canvas.create_oval(p.x - 3, y - 3, p.x + 3, y + 3, fill="red", outline="black")
            self.canvas.create_text(p.x + 5, y - 5, text=f"({p.x:.0f}, {p.y:.0f})", anchor=tk.SW, fill="black")


    def draw_all(self, points, left_edges, right_edges, hyperplane, convex_hull_pts, is_final_run=False):
        self.canvas.delete("all")
        def convert_y(y_coord): return self.height - y_coord

        if is_final_run:
            for edge in left_edges: 
                if isinstance(edge, Edge):
                    self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="black", width=1.5)
        else:
            for edge in left_edges: 
                if isinstance(edge, Edge):
                    self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="#a6d1e6", width=1.5)
            for edge in right_edges: 
                if isinstance(edge, Edge):
                    self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="#b5e5a4", width=1.5)
            for edge in hyperplane: 
                if isinstance(edge, Edge):
                    self.canvas.create_line(edge.start.x, convert_y(edge.start.y), edge.end.x, convert_y(edge.end.y), fill="blue", width=2)
        
        if len(convex_hull_pts) > 1:
            flat_pts = [c for p in convex_hull_pts for c in (p.x, convert_y(p.y))]
            if len(convex_hull_pts) == 2: self.canvas.create_line(flat_pts, fill="green", width=2)
            else: self.canvas.create_polygon(flat_pts, outline="green", fill="", width=2)
        
        for p in points:
            y = convert_y(p.y)
            self.canvas.create_oval(p.x - 3, y - 3, p.x + 3, y + 3, fill="red", outline="black")
            self.canvas.create_text(p.x + 5, y - 5, text=f"({p.x:.0f}, {p.y:.0f})", anchor=tk.SW, fill="black")


if __name__ == "__main__":
    Point.normal = lambda self: Point(-self.y, self.x)
    app = Application()
    app.mainloop()
