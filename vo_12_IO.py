# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk, messagebox
import math
import random
import sys
from collections import Counter
from tkinter import filedialog
# 增加遞迴深度限制，以處理較多點的情況
sys.setrecursionlimit(2000)

# --- 0. 輔助資料結構 ---

class StepRecord:
    """
    [MODIFIED] 用於儲存演算法每一步的狀態。
    - 新增 all_points: 儲存最一開始的所有點，用於繪製背景點。
    - 新增 left_hull, right_hull: 儲存合併前左右子集的凸包。
    - 新增 collision_info: 儲存超平面追蹤時的碰撞日誌。
    """
    def __init__(self, description, points, all_points=None, left_edges=None, right_edges=None, hyperplane=None, convex_hull=None, left_hull=None, right_hull=None, collision_info=None):
        self.description = description
        # Active points for this step
        self.points = [Point(p.x, p.y) for p in points]
        # All points in the original problem
        self.all_points = [Point(p.x, p.y) for p in all_points] if all_points else self.points
        self.left_edges = [Edge(e.start, e.end) for e in left_edges] if left_edges else []
        self.right_edges = [Edge(e.start, e.end) for e in right_edges] if right_edges else []
        self.hyperplane = [Edge(e.start, e.end) for e in hyperplane] if hyperplane else []
        self.convex_hull = Polygon(points=[Point(p.x, p.y) for p in convex_hull.points]) if convex_hull and convex_hull.points else None
        self.left_hull = Polygon(points=[Point(p.x, p.y) for p in left_hull.points]) if left_hull and left_hull.points else None
        self.right_hull = Polygon(points=[Point(p.x, p.y) for p in right_hull.points]) if right_hull and right_hull.points else None
        self.collision_info = collision_info


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
    def __hash__(self): return hash((round(self.x, 5), round(self.y, 5)))
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
        
        # 找到 a 的最右點和 b 的最左點
        idx_a, idx_b = max(range(a_size), key=lambda i: a_pts[i].x), min(range(b_size), key=lambda i: b_pts[i].x)
        
        # --- 計算上公切線 ---
        upper_a, upper_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            # [FIX] 將 >= 和 <= 改為 > 和 <，以正確處理共線點，避免震盪
            while Polygon._orientation(b_pts[upper_b], a_pts[upper_a], a_pts[(upper_a + 1) % a_size]) > 0:
                upper_a = (upper_a + 1) % a_size
                done = False
                    
            while Polygon._orientation(a_pts[upper_a], b_pts[upper_b], b_pts[(b_size + upper_b - 1) % b_size]) < 0:
                upper_b = (b_size + upper_b - 1) % b_size
                done = False
        
        # --- 計算下公切線 ---
        lower_a, lower_b = idx_a, idx_b
        done = False
        while not done:
            done = True
            # [FIX] 將 >= 和 <= 改為 > 和 <，以正確處理共線點，避免震盪
            while Polygon._orientation(a_pts[lower_a], b_pts[lower_b], b_pts[(lower_b + 1) % b_size]) > 0:
                lower_b = (lower_b + 1) % b_size
                done = False
            
            while Polygon._orientation(b_pts[lower_b], a_pts[lower_a], a_pts[(a_size + lower_a - 1) % a_size]) < 0:
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
        edges, hull, h_planes = self._divide_and_conquer(sorted_points, self.points, step_mode)
        
        if not step_mode:
            final_edges = self._clip_edges(edges)
            final_h_planes = self._clip_edges(h_planes) if h_planes else []
            return final_edges, hull, final_h_planes
        return None, None, None

    def _divide_and_conquer(self, points, all_points, step_mode):
        n = len(points)
        if n <= 1: 
            return [], Polygon.build_hull(points), []

        if n <= 3:
            edges, hull, h_planes = self._voronoi_base(points)
            if step_mode:
                clipped_edges = self._clip_edges(edges)
                self.steps.append(StepRecord(f"基礎案例: 計算 {n} 個點", points, all_points=all_points, left_edges=clipped_edges, convex_hull=hull))
            return edges, hull, h_planes

        mid = n // 2
        left_points, right_points = points[:mid], points[mid:]
        
        left_edges, left_hull, left_hps = self._divide_and_conquer(left_points, all_points, step_mode)
        right_edges, right_hull, right_hps = self._divide_and_conquer(right_points, all_points, step_mode)
        
        merged_edges, new_h_plane = self._merge(
            (left_edges, left_hull, left_points), 
            (right_edges, right_hull, right_points), 
            all_points,
            step_mode
        )
        
        final_hull = Polygon.build_hull(points)
        all_h_planes = (left_hps if left_hps else []) + (right_hps if right_hps else []) + (new_h_plane if new_h_plane else [])
        
        if step_mode:
            l_edges, r_edges, h_edges = merged_edges
            clipped_l = self._clip_edges(l_edges)
            clipped_r = self._clip_edges(r_edges)
            clipped_h = self._clip_edges(h_edges)
            self.steps.append(StepRecord(
                f"完成合併: {len(points)} 個點", 
                points, 
                all_points=all_points,
                left_edges=clipped_l, 
                right_edges=clipped_r, 
                hyperplane=clipped_h, 
                convex_hull=final_hull
            ))
            return l_edges + r_edges + h_edges, final_hull, all_h_planes
        
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

    def _merge(self, left_data, right_data, all_points, step_mode):
        left_edges, left_hull, left_points = left_data
        right_edges, right_hull, right_points = right_data
        
        ua_idx, ub_idx, la_idx, lb_idx = Polygon.merge_convex_hulls(left_hull, right_hull)
        if not left_hull.points or not right_hull.points: 
            return left_edges + right_edges, []
        
        p_ua, p_ub = left_hull[ua_idx], right_hull[ub_idx]
        p_la, p_lb = left_hull[la_idx], right_hull[lb_idx]

        if step_mode:
            merged_hull = Polygon.build_hull(left_points + right_points)
            self.steps.append(StepRecord(
                "步驟 1/5: 合併凸包", 
                points=left_points + right_points, 
                all_points=all_points,
                left_edges=self._clip_edges(left_edges), 
                right_edges=self._clip_edges(right_edges), 
                convex_hull=merged_hull,
                left_hull=left_hull,
                right_hull=right_hull,
                collision_info="找到左右凸包的上下公切線。"
            ))

        hyperplane, scan_trends = [], []
        scan_line = Line(p_ua, p_ub)
        lower_tangent = Line(p_la, p_lb)
        
        l_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in left_edges]
        r_edges = [Line(e.start, e.end, e.norm_start, e.norm_end) for e in right_edges]
        
        last_v_pt = None
        loop_count = 0
        prev_used_edge_info = None

        while True:
            loop_count += 1
            if loop_count > (len(left_points) + len(right_points)) * 4: break
            
            bisector = scan_line.get_perpendicular_bisector()
            search_origin = last_v_pt
            if search_origin is None:
                clipped_pts = self._cohen_sutherland_clip(bisector.start, bisector.end)
                if clipped_pts:
                    p1_clip, p2_clip = clipped_pts
                    search_origin = p1_clip if p1_clip.y > p2_clip.y else p2_clip
                else: break

            l_is_pt, l_is_idx = self._find_near_intra_is(bisector, l_edges, search_origin)
            r_is_pt, r_is_idx = self._find_near_intra_is(bisector, r_edges, search_origin)
            near_is_pt, choose_dir, near_is_idx = self._find_near_inter_is(bisector, l_is_pt, r_is_pt, l_is_idx, r_is_idx)

            start_pt = last_v_pt if last_v_pt is not None else search_origin
            is_lower = Point.is_similar(scan_line.norm_start, lower_tangent.norm_start) and Point.is_similar(scan_line.norm_end, lower_tangent.norm_end)

            if near_is_pt is None or is_lower:
                bisector.to_direction('d')
                direction_vector = bisector.end - bisector.start
                end_pt = start_pt + direction_vector * (self.width + self.height)
                segment = Line(start_pt, end_pt, scan_line.norm_start, scan_line.norm_end)
                hyperplane.append(segment)
                scan_trends.append(Trend(direction=choose_dir, index=near_is_idx))

                if step_mode:
                    info = "已到達下公切線，或找不到下一個交點。\n新增最後一條射線。"
                    self.steps.append(StepRecord(
                        f"步驟 2/5: 追蹤超平面 (段 {len(hyperplane)})",
                        points=left_points + right_points, all_points=all_points,
                        left_edges=self._clip_edges(l_edges), right_edges=self._clip_edges(r_edges),
                        hyperplane=self._clip_edges(hyperplane), convex_hull=Polygon.build_hull(left_points + right_points),
                        collision_info=info
                    ))
                break 
            
            end_pt = near_is_pt
            last_v_pt = end_pt

            segment = Line(start_pt, end_pt, scan_line.norm_start, scan_line.norm_end)
            hyperplane.append(segment)
            scan_trends.append(Trend(direction=choose_dir, index=near_is_idx))

            if step_mode:
                side = 'L' if choose_dir == 1 else 'R'
                info_pt_str = f"P({end_pt.x:.1f}, {end_pt.y:.1f})"
                info = f"超平面撞到 {side}{near_is_idx} 於點 {info_pt_str}"
                self.steps.append(StepRecord(
                    f"步驟 2/5: 追蹤超平面 (段 {len(hyperplane)})",
                    points=left_points + right_points, all_points=all_points,
                    left_edges=self._clip_edges(l_edges), right_edges=self._clip_edges(r_edges),
                    hyperplane=self._clip_edges(hyperplane), convex_hull=Polygon.build_hull(left_points + right_points),
                    collision_info=info
                ))

            if prev_used_edge_info:
                edge_list, edge_index = prev_used_edge_info
                if 0 <= edge_index < len(edge_list): edge_list[edge_index].used = False
                prev_used_edge_info = None

            if choose_dir == 1 and near_is_idx != -1: 
                l_edges[near_is_idx].used = True
                prev_used_edge_info = (l_edges, near_is_idx)
            elif choose_dir == 2 and near_is_idx != -1: 
                r_edges[near_is_idx].used = True
                prev_used_edge_info = (r_edges, near_is_idx)

            scan_line = self._move_scan_next_loop(scan_line, l_edges, r_edges, l_is_idx, r_is_idx, choose_dir)

        self._update_trend(hyperplane, scan_trends)
        pruned_vertices_hashes = self._elimination(hyperplane, scan_trends, l_edges, r_edges)
        
        if step_mode: 
            self.steps.append(StepRecord(
                "步驟 3/5: 執行消線(裁切)", 
                points=left_points + right_points, 
                all_points=all_points,
                left_edges=self._clip_edges(l_edges), 
                right_edges=self._clip_edges(r_edges), 
                hyperplane=self._clip_edges(hyperplane),
                convex_hull=Polygon.build_hull(left_points + right_points),
                collision_info="根據超平面的轉向，修剪與其相交的舊邊緣。"
            ))

        # --- [MODIFICATION] Pass hyperplane to cascade delete ---
        l_edges, r_edges = self._cascade_delete_internal_edges(l_edges, r_edges, pruned_vertices_hashes, hyperplane)

        if step_mode:
             self.steps.append(StepRecord(
                "步驟 4/5: 執行連鎖刪除",
                points=left_points + right_points,
                all_points=all_points,
                left_edges=self._clip_edges(l_edges),
                right_edges=self._clip_edges(r_edges),
                hyperplane=self._clip_edges(hyperplane),
                convex_hull=Polygon.build_hull(left_points + right_points),
                collision_info="刪除因消線而產生的所有懸空內部邊緣。"
            ))
            
        final_l_edges = l_edges
        final_r_edges = r_edges
            
        if step_mode:
            return (final_l_edges, final_r_edges, hyperplane), hyperplane

        final_edges = final_l_edges + final_r_edges + hyperplane
        return final_edges, hyperplane
    
    def _find_near_intra_is(self, scan_pb, edges, search_origin):
        nearest_is_pt, nearest_is_idx = None, -1
        min_t = float('inf')

        spb = Line(scan_pb.start, scan_pb.end)
        if (spb.end.y - spb.start.y) > 0:
            spb.start, spb.end = spb.end, spb.start
        
        p1, p2 = spb.start, spb.end
        vec = p2 - p1
        vec_len_sq = vec.x**2 + vec.y**2
        if vec_len_sq < 1e-12:
            return None, -1

        origin_vec = search_origin - p1
        t_origin = (origin_vec.x * vec.x + origin_vec.y * vec.y) / vec_len_sq
        
        for i, edge in enumerate(edges):
            if hasattr(edge, 'used') and edge.used:
                continue
            
            p3, p4 = edge.start, edge.end

            den = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x)
            if abs(den) < 1e-9:
                continue

            t_num = (p1.x - p3.x) * (p3.y - p4.y) - (p1.y - p3.y) * (p3.x - p4.x)
            u_num = -((p1.x - p2.x) * (p1.y - p3.y) - (p1.y - p2.y) * (p1.x - p3.x))

            t = t_num / den
            u = u_num / den

            epsilon = 1e-9
            # [FIXED] Changed t > t_origin + epsilon / 10 to t > t_origin - epsilon
            # This allows detection of intersections at the search_origin (3-way vertices)
            # by creating a small tolerance window.
            if (0.0 - epsilon <= u <= 1.0 + epsilon) and (t > t_origin - epsilon): 
                if t < min_t:
                    min_t = t
                    x = p1.x + t * (p2.x - p1.x)
                    y = p1.y + t * (p2.y - p1.y)
                    nearest_is_pt = Point(x, y)
                    nearest_is_idx = i
                            
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
            new_start = edge_to_check.norm_end if Point.is_similar(new_start, edge_to_check.norm_start) else edge_to_check.norm_start
        elif choose == 2 and r_idx != -1:
            edge_to_check = r_edges[r_idx]
            new_end = edge_to_check.norm_end if Point.is_similar(new_end, edge_to_check.norm_start) else edge_to_check.norm_start
        
        return Line(new_start, new_end)

    def _update_trend(self, hyperplane, scan_trends):
        for i in range(len(hyperplane) - 1):
            v1 = hyperplane[i].slope
            v2 = hyperplane[i+1].slope
            cp = Point.cross_product(v1, v2)
            
            if cp > 1e-9:
                scan_trends[i].trend = 2 # 右轉 (順時針, CW)
            elif cp < -1e-9:
                scan_trends[i].trend = 1 # 左轉 (逆時針, CCW)

    def _elimination(self, hyperplane, scan_trends, l_edges, r_edges):
        pruning_map = {}
        # --- [MODIFICATION] This set will store the hashes of pruned vertices ---
        pruned_vertices_hashes = set()

        for i in range(len(hyperplane) - 1):
            edge_info = scan_trends[i]
            intersection_pt = hyperplane[i].end

            if edge_info.direction == 0 or intersection_pt is None:
                continue

            edges_list = l_edges if edge_info.direction == 1 else r_edges
            if not (0 <= edge_info.index < len(edges_list)):
                continue
            
            edge_to_prune = edges_list[edge_info.index]

            key = (edge_info.direction, edge_info.index)
            if key not in pruning_map:
                pruning_map[key] = []
            
            is_duplicate = False
            for existing_pt in pruning_map[key]:
                if Point.is_similar(existing_pt, intersection_pt):
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                pruning_map[key].append(intersection_pt)

        for key, intersections in pruning_map.items():
            direction, index = key
            edges_list = l_edges if direction == 1 else r_edges
            edge_to_prune = edges_list[index]

            if len(intersections) == 1:
                intersection_pt = intersections[0]
                
                original_turn = None
                v_hyper = None
                for i in range(len(hyperplane) - 1):
                    if scan_trends[i].direction == direction and \
                       scan_trends[i].index == index and \
                       Point.is_similar(hyperplane[i].end, intersection_pt):
                        original_turn = scan_trends[i]
                        v_hyper = hyperplane[i].slope
                        break
                
                if original_turn is None or v_hyper is None or original_turn.trend == 0:
                    continue

                v_to_end = edge_to_prune.end - intersection_pt
                v_to_start = edge_to_prune.start - intersection_pt
                cross_prod_end = Point.cross_product(v_hyper, v_to_end)
                cross_prod_start = Point.cross_product(v_hyper, v_to_start)
                
                # --- [MODIFICATION] Add the hash of the old, pruned vertex ---
                original_start_hash = edge_to_prune.start.__hash__()
                original_end_hash = edge_to_prune.end.__hash__()

                if original_turn.trend == 1: # Left turn (CCW)
                    if cross_prod_end < cross_prod_start:
                        pruned_vertices_hashes.add(original_end_hash)
                        edge_to_prune.end = intersection_pt
                    else:
                        pruned_vertices_hashes.add(original_start_hash)
                        edge_to_prune.start = intersection_pt
                elif original_turn.trend == 2: # Right turn (CW)
                    if cross_prod_end > cross_prod_start:
                        pruned_vertices_hashes.add(original_end_hash)
                        edge_to_prune.end = intersection_pt
                    else:
                        pruned_vertices_hashes.add(original_start_hash)
                        edge_to_prune.start = intersection_pt

            elif len(intersections) >= 2:
                pruned_vertices_hashes.add(edge_to_prune.start.__hash__())
                pruned_vertices_hashes.add(edge_to_prune.end.__hash__())
                p1, p2 = None, None
                if len(intersections) == 2:
                    p1, p2 = intersections[0], intersections[1]
                else: 
                    max_dist_sq = -1
                    for i in range(len(intersections)):
                        for j in range(i + 1, len(intersections)):
                            dist_sq = (intersections[i] - intersections[j]).length_sq()
                            if dist_sq > max_dist_sq:
                                max_dist_sq = dist_sq
                                p1, p2 = intersections[i], intersections[j]
                
                if p1 and p2:
                    edge_to_prune.start = p1
                    edge_to_prune.end = p2
        
        # --- [MODIFICATION] Return the collected hashes ---
        return pruned_vertices_hashes

    def _cascade_delete_internal_edges(self, l_edges, r_edges, pruned_vertices_hashes, hyperplane):
        """
        [MODIFIED] Complete rewrite of this function to implement BFS cascade deletion.
        It starts from pruned vertices and deletes connected edges until it hits an "anchor"
        (the new hyperplane or the bounding box).
        """
        if not pruned_vertices_hashes:
            return l_edges, r_edges

        all_edges = l_edges + r_edges
        adj = {} 
        hash_to_point = {}

        # 1. Build adjacency list and a hash-to-point mapping for all old edges
        for i, edge in enumerate(all_edges):
            for p in [edge.start, edge.end]:
                p_hash = p.__hash__()
                if p_hash not in adj:
                    adj[p_hash] = []
                    hash_to_point[p_hash] = p
                adj[p_hash].append(i)

        # 2. Identify "anchor" vertices from the new hyperplane. These stop the deletion.
        anchor_hashes = set()
        for edge in hyperplane:
            anchor_hashes.add(edge.start.__hash__())
            anchor_hashes.add(edge.end.__hash__())

        # 3. BFS to find all deletable edges
        queue = list(pruned_vertices_hashes)
        visited_hashes = set(pruned_vertices_hashes)
        deleted_edge_indices = set()

        while queue:
            vertex_hash = queue.pop(0)
            vertex_point = hash_to_point.get(vertex_hash)
            
            # STOPPING CONDITIONS for the BFS path:
            # - Stop if the vertex is part of the new hyperplane (it's an anchor).
            # - Stop if the vertex is on the main computational boundary (it's a valid ray).
            if vertex_hash in anchor_hashes or \
               (vertex_point and vertex_point.is_on_boundary(self.width, self.height)):
                continue
            
            # If the vertex has no connections in the old graph, stop.
            if vertex_hash not in adj:
                continue

            # Process all connected old edges
            for edge_idx in adj[vertex_hash]:
                if edge_idx in deleted_edge_indices:
                    continue
                
                # Mark this edge for deletion
                deleted_edge_indices.add(edge_idx)
                
                # Find the endpoint on the other side to continue the search
                edge = all_edges[edge_idx]
                other_vertex_hash = edge.end.__hash__() if edge.start.__hash__() == vertex_hash else edge.start.__hash__()
                
                if other_vertex_hash not in visited_hashes:
                    visited_hashes.add(other_vertex_hash)
                    queue.append(other_vertex_hash)

        # 4. Reconstruct the edge lists, keeping only the non-deleted edges
        final_l_edges, final_r_edges = [], []
        num_l_edges = len(l_edges)
        for i, edge in enumerate(all_edges):
            if i not in deleted_edge_indices:
                if i < num_l_edges:
                    final_l_edges.append(edge)
                else:
                    final_r_edges.append(edge)
                    
        return final_l_edges, final_r_edges

    def _clip_edges(self, edges, prefix="Edge"):
        if not edges:
            return []
        clipped = []
        for i, edge in enumerate(edges):
            if not isinstance(edge, Edge):
                continue
            
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
        self.visible_width = width
        self.visible_height = height
        
        self.comp_width = self.visible_width * 10
        self.comp_height = self.visible_height * 10
        
        self.comp_offset_x = (self.comp_width - self.visible_width) / 2
        self.comp_offset_y = (self.comp_height - self.visible_height) / 2

        self.title("Voronoi Diagram 600x600")
        self.geometry(f"{self.visible_width + 250}x{self.visible_height + 20}")
        self.resizable(False, False)
        
        self.points, self.voronoi_steps, self.current_step = [], [], 0
        self.autoplay_job = None
        self.all_test_cases = []
        self.current_test_case_index = -1
        
        self._create_widgets()

    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main_frame, width=self.visible_width, height=self.visible_height, bg="white", highlightthickness=1, highlightbackground="black")
        self.canvas.pack(side=tk.LEFT, padx=(0, 10), fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.add_point_on_click)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # --- Section 1: 新增站點 ---
        point_frame = ttk.LabelFrame(control_frame, text="1. Manually add points", padding=10)
        point_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(point_frame, text="Randomly generate points：").pack()
        self.random_points_entry = ttk.Entry(point_frame)
        self.random_points_entry.pack(fill=tk.X, pady=(0, 5))
        self.random_points_entry.insert(0, "6")
        ttk.Button(point_frame, text="Generate", command=self.generate_random_points).pack(fill=tk.X)
        
        coord_frame = ttk.Frame(point_frame)
        coord_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(coord_frame, text="X:").pack(side=tk.LEFT, padx=(0, 2))
        self.x_entry = ttk.Entry(coord_frame, width=6)
        self.x_entry.pack(side=tk.LEFT)
        
        ttk.Label(coord_frame, text="Y:").pack(side=tk.LEFT, padx=(5, 2))
        self.y_entry = ttk.Entry(coord_frame, width=6)
        self.y_entry.pack(side=tk.LEFT)
        
        self.add_coord_button = ttk.Button(coord_frame, text="Add", command=self.add_point_from_entry)
        self.add_coord_button.pack(side=tk.LEFT, padx=(5, 0))
        self.y_entry.bind("<Return>", lambda event: self.add_coord_button.invoke())

        
        ttk.Button(point_frame, text="Clear", command=self.clear_canvas).pack(fill=tk.X, pady=(5,0))

        # --- Section 2: 測資瀏覽 ---
        case_nav_frame = ttk.LabelFrame(control_frame, text="2. File of test data", padding=10)
        case_nav_frame.pack(fill=tk.X, pady=(0, 10))
        
        case_buttons_frame = ttk.Frame(case_nav_frame)
        case_buttons_frame.pack(fill=tk.X)

        ttk.Button(case_buttons_frame, text="Load file(.txt)", command=self.load_points_from_file).pack(fill=tk.X, pady=(5,0))

        self.prev_case_button = ttk.Button(case_buttons_frame, text="Previous date", command=self.prev_test_case, state=tk.DISABLED)
        self.prev_case_button.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        self.next_case_button = ttk.Button(case_buttons_frame, text="Next data", command=self.next_test_case, state=tk.DISABLED)
        self.next_case_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5,0))
        
        self.case_label = ttk.Label(case_nav_frame, text="", anchor="center")
        self.case_label.pack(fill=tk.X, pady=5)

        # --- Section 3: 演算法控制 ---
        step_frame = ttk.LabelFrame(control_frame, text="3.  Algorithm control", padding=10)
        step_frame.pack(fill=tk.X, pady=(0, 10))
        
        nav_frame = ttk.Frame(step_frame)
        nav_frame.pack(fill=tk.X)

        self.prev_button = ttk.Button(nav_frame, text="Previous step", command=self.prev_step, state=tk.DISABLED)
        self.prev_button.pack(side=tk.LEFT, expand=True, fill=tk.X)
        
        self.next_button = ttk.Button(nav_frame, text="Step by step", command=self.next_step)
        self.next_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5,0))

        self.autoplay_button = ttk.Button(step_frame, text="Run", command=self.start_autoplay)
        self.autoplay_button.pack(fill=tk.X, pady=(5, 0))

        self.step_label = ttk.Label(step_frame, text="", anchor="center")
        self.step_label.pack(fill=tk.X, pady=5)

        # --- Section 5: 檔案操作 ---
        export_frame = ttk.LabelFrame(control_frame, text="4. File operations", padding=10)
        export_frame.pack(fill=tk.X)
        ttk.Button(export_frame, text="Export points and line segments", command=self.export_to_file).pack(fill=tk.X)
        ttk.Button(export_frame, text="Importing graphics from a file", command=self.import_from_file).pack(fill=tk.X, pady=(5,0))


    def _reset_to_custom_mode(self):
        if self.current_test_case_index != -1:
            self.all_test_cases.clear()
            self.current_test_case_index = -1
            self.points.clear()
            self.canvas.delete("all")
            self.case_label.config(text="")
            self.prev_case_button.config(state=tk.DISABLED)
            self.next_case_button.config(state=tk.DISABLED)

    def add_point_on_click(self, event):
        self._reset_to_custom_mode()
        comp_x = event.x + self.comp_offset_x
        comp_y = (self.visible_height - event.y) + self.comp_offset_y
        self.points.append(Point(comp_x, comp_y))
        self.reset_steps()
        self.canvas.delete("all")
        self.draw_points(self.points, self.points)

    def add_point_from_entry(self):
        try:
            x_val = float(self.x_entry.get())
            y_val = float(self.y_entry.get())
            
            if 0 <= x_val <= self.visible_width and 0 <= y_val <= self.visible_height:
                self._reset_to_custom_mode()
                comp_x = x_val + self.comp_offset_x
                comp_y = y_val + self.comp_offset_y
                self.points.append(Point(comp_x, comp_y))
                self.reset_steps()
                self.canvas.delete("all")
                self.draw_points(self.points, self.points)
                self.x_entry.delete(0, tk.END)
                self.y_entry.delete(0, tk.END)
                self.x_entry.focus()
            else:
                messagebox.showwarning("座標超出範圍", f"請輸入在畫布範圍內的座標 (X: 0-{self.visible_width}, Y: 0-{self.visible_height})。")
        except (ValueError, TypeError):
            messagebox.showerror("無效輸入", "請在 X 和 Y 座標欄位中輸入有效的數字。")

    def generate_random_points(self):
        self.clear_canvas()
        try:
            num = int(self.random_points_entry.get())
            for _ in range(num):
                rand_x_vis = random.randint(10, self.visible_width - 10)
                rand_y_vis = random.randint(10, self.visible_height - 10)
                self.points.append(Point(rand_x_vis + self.comp_offset_x, rand_y_vis + self.comp_offset_y))
            self.case_label.config(text="")
        except (ValueError, TypeError): messagebox.showerror("無效輸入", "請輸入一個有效的整數。")
        self.draw_points(self.points, self.points)
        
    def _parse_test_cases(self, lines):
        test_cases = []
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if not line:
                i += 1
                continue
            try:
                num_points = int(line)
                if num_points == 0: break 
                if i + num_points >= len(lines): break 
                
                points = []
                for j in range(1, num_points + 1):
                    point_line = lines[i + j].strip().split()
                    if len(point_line) >= 2:
                        x, y = float(point_line[0]), float(point_line[1])
                        points.append(Point(x, y))
                
                if len(points) == num_points:
                    case_name = f"案例 {len(test_cases) + 1} ({num_points} 個點)"
                    test_cases.append({"name": case_name, "points": points})
                
                i += num_points + 1
            except ValueError:
                i += 1
        return test_cases

    def load_points_from_file(self):
        file_path = filedialog.askopenfilename(
            title="選擇一個測資檔案",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not file_path: return

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            test_cases = self._parse_test_cases(lines)
            if not test_cases:
                messagebox.showwarning("無有效測資", "在檔案中找不到任何有效的測資。")
                return

            self.clear_canvas()
            self.all_test_cases = test_cases
            self.current_test_case_index = 0
            self._load_current_test_case()

        except Exception as e:
            messagebox.showerror("讀取失敗", f"讀取或解析檔案時發生錯誤: {e}")

    def _load_current_test_case(self):
        if not self.all_test_cases or not (0 <= self.current_test_case_index < len(self.all_test_cases)):
            return

        self.points.clear()
        self.reset_steps()
        self.canvas.delete("all")
        
        case_data = self.all_test_cases[self.current_test_case_index]
        self.points = [Point(p.x + self.comp_offset_x, p.y + self.comp_offset_y) for p in case_data["points"]]
        
        self.draw_points(self.points, self.points)
        
        total_cases = len(self.all_test_cases)
        self.case_label.config(text=f"測資 {self.current_test_case_index + 1}/{total_cases}")
        self.prev_case_button.config(state=tk.NORMAL if self.current_test_case_index > 0 else tk.DISABLED)
        self.next_case_button.config(state=tk.NORMAL if self.current_test_case_index < total_cases - 1 else tk.DISABLED)

    def next_test_case(self):
        if self.current_test_case_index < len(self.all_test_cases) - 1:
            self.current_test_case_index += 1
            self._load_current_test_case()

    def prev_test_case(self):
        if self.current_test_case_index > 0:
            self.current_test_case_index -= 1
            self._load_current_test_case()

    def _filter_isolated_edges(self, edges, l_count=None, r_count=None):
        if not edges: 
            return []

        endpoint_counts = Counter()
        for edge in edges:
            endpoint_counts[edge.start.__hash__()] += 1
            endpoint_counts[edge.end.__hash__()] += 1

        kept_edges = []
        for edge in edges:
            start_degree = endpoint_counts.get(edge.start.__hash__(), 0)
            end_degree = endpoint_counts.get(edge.end.__hash__(), 0)

            is_start_on_boundary = edge.start.is_on_boundary(self.comp_width, self.comp_height)
            is_end_on_boundary = edge.end.is_on_boundary(self.comp_width, self.comp_height)

            # Condition for DELETION:
            # The edge is deleted only if both its endpoints have a degree of 1 (only connected
            # to this edge) AND neither of them lies on the main computational boundary.
            # This correctly identifies truly floating segments.
            is_truly_isolated = (start_degree == 1 and end_degree == 1 and 
                                 not is_start_on_boundary and not is_end_on_boundary)

            if not is_truly_isolated:
                kept_edges.append(edge)
                
        return kept_edges

    def run_normal(self):
        if len(self.points) < 2:
            messagebox.showinfo("資訊", "請先在畫布上新增至少兩個點。")
            return
        
        self.reset_steps()
        voronoi = VoronoiDiagram(self.points, self.comp_width, self.comp_height)
        
        edges, hull, h_planes = voronoi.run(step_mode=False)
        
        if edges is None:
            messagebox.showerror("執行錯誤", "演算法執行過程中發生錯誤。")
            return
        
        all_segments_to_check = edges + (h_planes if h_planes else [])
        kept_edges = self._filter_isolated_edges(all_segments_to_check)
        
        final_step = StepRecord(
            description="最終結果",
            points=self.points,
            all_points=self.points,
            left_edges=kept_edges,
            convex_hull=hull
        )
        self.draw_all(final_step, is_final_run=True)

    def run_step_by_step_generate(self):
        """
        [MODIFIED] This is now a helper function called by next_step() on the first press.
        It no longer configures buttons directly; update_step_display() handles that.
        """
        self.reset_steps()
        voronoi = VoronoiDiagram(self.points, self.comp_width, self.comp_height)
        voronoi.run(step_mode=True)
        self.voronoi_steps = voronoi.steps
        
        if self.voronoi_steps:
            last_step = self.voronoi_steps[-1]
            all_segments_before_cleanup = last_step.left_edges + last_step.right_edges + last_step.hyperplane
            final_kept_edges = self._filter_isolated_edges(
                all_segments_before_cleanup, 
                l_count=len(last_step.left_edges), 
                r_count=len(last_step.right_edges)
            )
            
            cleanup_step = StepRecord(
                description="最終清理：移除懸空邊",
                points=last_step.points,
                all_points=self.points,
                left_edges=final_kept_edges,
                right_edges=[],
                hyperplane=[],
                convex_hull=Polygon(points=last_step.convex_hull.points) if last_step.convex_hull else None,
                collision_info="最終結果圖。"
            )
            self.voronoi_steps.append(cleanup_step)

            self.current_step = 0
            self.update_step_display()

    def save_points_to_file(self):
        file_name = "無註解Voronoi Diagra公開測資.txt"
        if not self.points:
            messagebox.showinfo("沒有點", "畫布上沒有點可以儲存。")
            return

        try:
            lines = []
            try:
                with open(file_name, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                if lines and lines[-1].strip() == '0': lines.pop()
            except FileNotFoundError: pass

            with open(file_name, 'w', encoding='utf-8') as f:
                f.writelines(lines)
                f.write(f"{len(self.points)}\n")
                for p_comp in self.points:
                    vis_x = p_comp.x - self.comp_offset_x
                    vis_y = p_comp.y - self.comp_offset_y
                    f.write(f"{int(vis_x)} {int(vis_y)}\n")
                f.write("0\n")

            messagebox.showinfo("儲存成功", f"已成功將 {len(self.points)} 個點附加到 {file_name}")
        except Exception as e:
            messagebox.showerror("儲存失敗", f"寫入檔案時發生錯誤: {e}")

    def next_step(self):
        """
        [MODIFIED] Handles step generation on first press, and advances on subsequent presses.
        The timed auto-advance feature is removed from here and handled by start_autoplay.
        """
        # If steps haven't been generated, the first press will generate them.
        if not self.voronoi_steps:
            if len(self.points) < 2:
                messagebox.showinfo("資訊", "請先在畫布上新增至少兩個點。")
                return
            # Call the generation function. It creates steps and shows step 0.
            self.run_step_by_step_generate() 
            return # The work for the first press is done.

        # Logic for subsequent presses (advancing the step).
        self.stop_autoplay()
        if self.current_step < len(self.voronoi_steps) - 1:
            self.current_step += 1
            self.update_step_display()

    def prev_step(self):
        self.stop_autoplay()
        if self.current_step > 0:
            self.current_step -= 1
            self.update_step_display()
            
    def start_autoplay(self):
        """
        [MODIFIED] Generates steps if they don't exist, then starts autoplaying.
        This function now contains all timing logic for autoplay.
        """
        if self.autoplay_job:
            return

        # --- Step 1: Generate steps if they don't exist ---
        if not self.voronoi_steps:
            if len(self.points) < 2:
                messagebox.showinfo("資訊", "請先在畫布上新增至少兩個點。")
                return
            # This populates self.voronoi_steps and shows the first step.
            self.run_step_by_step_generate()

        # --- Step 2: Define and start the autoplay task ---
        def autoplay_task():
            if self.current_step < len(self.voronoi_steps) - 1:
                # Advance one step
                self.current_step += 1
                self.update_step_display()
                # Schedule the next iteration of the autoplay task.
                self.autoplay_job = self.after(0, autoplay_task)
            else:
                # Reached the end.
                self.stop_autoplay()

        # --- Step 3: Set UI state and launch the task ---
        self.prev_button.config(state=tk.DISABLED)
        self.next_button.config(state=tk.DISABLED)
        #self.autoplay_button.config(state=tk.DISABLED)
        
        # Launch the first iteration of the task.
        self.autoplay_job = self.after(0, autoplay_task)

    def stop_autoplay(self):
        """如果正在自動播放，則停止它"""
        if self.autoplay_job:
            self.after_cancel(self.autoplay_job)
            self.autoplay_job = None
        # Always update the button states when stopping.
        self.update_step_display()

    def update_step_display(self):
        # --- Update Step Label and Buttons ---
        if not self.voronoi_steps: 
            self.step_label.config(text="點擊「下一步」開始")
            self.prev_button.config(state=tk.DISABLED)
            self.next_button.config(state=tk.NORMAL)
            
            self.info_text.config(state=tk.NORMAL)
            self.info_text.delete("1.0", tk.END)
            self.info_text.insert("1.0", "在此處顯示詳細的步驟資訊。")
            self.info_text.config(state=tk.DISABLED)
            return

        step_record = self.voronoi_steps[self.current_step]
        self.step_label.config(text=f"第 {self.current_step + 1}/{len(self.voronoi_steps)} 步\n{step_record.description}")
        self.draw_all(step_record, is_final_run=False)

        if self.autoplay_job:
            self.prev_button.config(state=tk.DISABLED)
            self.next_button.config(state=tk.DISABLED)
        else:
            is_not_last_step = self.current_step < len(self.voronoi_steps) - 1
            self.prev_button.config(state=tk.NORMAL if self.current_step > 0 else tk.DISABLED)
            self.next_button.config(state=tk.NORMAL if is_not_last_step else tk.DISABLED)

        # --- Update Info Text Box ---
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete("1.0", tk.END)

        def to_vis(p):
            vis_x = p.x - self.comp_offset_x
            vis_y = p.y - self.comp_offset_y
            return f"({vis_x:.0f}, {vis_y:.0f})"

        info_content = []
        if step_record.collision_info:
            info_content.append(f"[日誌]\n{step_record.collision_info}\n")

        info_content.append(f"--- 左側線段 (L) [{len(step_record.left_edges)}] ---")
        if step_record.left_edges:
            for i, edge in enumerate(step_record.left_edges): info_content.append(f"L{i}: {to_vis(edge.start)} -> {to_vis(edge.end)}")
        else: info_content.append("(無)")
        info_content.append("")

        info_content.append(f"--- 右側線段 (R) [{len(step_record.right_edges)}] ---")
        if step_record.right_edges:
            for i, edge in enumerate(step_record.right_edges): info_content.append(f"R{i}: {to_vis(edge.start)} -> {to_vis(edge.end)}")
        else: info_content.append("(無)")
        info_content.append("")
        
        info_content.append(f"--- 超平面線段 (H) [{len(step_record.hyperplane)}] ---")
        if step_record.hyperplane:
            for i, edge in enumerate(step_record.hyperplane): info_content.append(f"H{i}: {to_vis(edge.start)} -> {to_vis(edge.end)}")
        else: info_content.append("(無)")

        self.info_text.insert("1.0", "\n".join(info_content))
        self.info_text.config(state=tk.DISABLED)


    def clear_canvas(self):
        self.points.clear()
        self.reset_steps()
        
        self.all_test_cases.clear()
        self.current_test_case_index = -1
        self.case_label.config(text="未載入檔案")
        self.prev_case_button.config(state=tk.DISABLED)
        self.next_case_button.config(state=tk.DISABLED)
        
        self.canvas.delete("all")
        self.update_step_display()

    def reset_steps(self):
        self.stop_autoplay()
        self.voronoi_steps.clear()
        self.current_step = 0
        if hasattr(self, 'step_label'):
            self.update_step_display()

    def draw_points(self, all_points, active_points):
        """
        [MODIFIED] Draws all points, highlighting the active ones.
        - all_points: All points in the problem.
        - active_points: The subset of points currently being processed.
        """
        active_point_coords = set((p.__hash__()) for p in active_points)

        for p_comp in all_points:
            is_active = p_comp.__hash__() in active_point_coords
            
            fill_color = "red" if is_active else "#cccccc" # Gray for inactive points
            text_color = "black" if is_active else "#999999"
            
            vis_x = p_comp.x - self.comp_offset_x
            vis_y = p_comp.y - self.comp_offset_y
            
            if 0 <= vis_x <= self.visible_width and 0 <= vis_y <= self.visible_height:
                canvas_y = self.visible_height - vis_y
                self.canvas.create_oval(vis_x - 3, canvas_y - 3, vis_x + 3, canvas_y + 3, fill=fill_color, outline="black")
                if is_active:
                    self.canvas.create_text(vis_x + 5, canvas_y - 5, text=f"({vis_x:.0f}, {vis_y:.0f})", anchor=tk.SW, fill=text_color)
    
    def _clip_to_visible_canvas(self, p1, p2):
        INSIDE, LEFT, RIGHT, BOTTOM, TOP = 0, 1, 2, 4, 8
        xmin, ymin, xmax, ymax = 0, 0, self.visible_width, self.visible_height
        
        def _get_code(p):
            code = INSIDE
            if p.x < xmin: code |= LEFT
            elif p.x > xmax: code |= RIGHT
            if p.y < ymin: code |= BOTTOM
            elif p.y > ymax: code |= TOP
            return code

        code1, code2 = _get_code(p1), _get_code(p2)
        while True:
            if not (code1 | code2): return (p1, p2)
            if code1 & code2: return None
            
            code_out = code1 or code2
            x, y = 0.0, 0.0
            dy, dx = p2.y - p1.y, p2.x - p1.x
            
            if code_out & TOP:
                x = p1.x + dx * (ymax - p1.y) / dy if dy != 0 else p1.x
                y = ymax
            elif code_out & BOTTOM:
                x = p1.x + dx * (ymin - p1.y) / dy if dy != 0 else p1.x
                y = ymin
            elif code_out & RIGHT:
                y = p1.y + dy * (xmax - p1.x) / dx if dx != 0 else p1.y
                x = xmax
            elif code_out & LEFT:
                y = p1.y + dy * (xmin - p1.x) / dx if dx != 0 else p1.y
                x = xmin
            
            if code_out == code1:
                p1, code1 = Point(x, y), _get_code(Point(x, y))
            else:
                p2, code2 = Point(x, y), _get_code(Point(x, y))

    def _get_final_edges(self):
        """
        [NEW] Helper to get the final, cleaned list of Voronoi edges.
        If a step-by-step run was completed, it uses the result of the last step.
        Otherwise, it runs the algorithm in the background to get the final result.
        """
        # Check if the final "cleanup" step is available
        if self.voronoi_steps and self.voronoi_steps[-1].description.startswith("最終清理"):
            return self.voronoi_steps[-1].left_edges

        # If not, compute it directly
        if len(self.points) < 2:
            return []
            
        voronoi = VoronoiDiagram(self.points, self.comp_width, self.comp_height)
        edges, _, _ = voronoi.run(step_mode=False)
        
        if edges is None:
            return []

        kept_edges = self._filter_isolated_edges(edges)
        return kept_edges

    def export_to_file(self):
        """
        [NEW] Saves the visible points and final, clipped Voronoi edges to a text file.
        """
        if not self.points:
            messagebox.showinfo("無內容可匯出", "畫布上沒有點。請先新增點。")
            return
            
        final_edges = self._get_final_edges()
        if not final_edges:
            messagebox.showwarning("無結果可匯出", "尚未產生最終的 Voronoi 圖。請先執行演算法。")
            return
            
        file_path = filedialog.asksaveasfilename(
            title="儲存點和線段",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if not file_path:
            return

        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                # Write points (in visible coordinates)
                for p_comp in self.points:
                    vis_x = p_comp.x - self.comp_offset_x
                    vis_y = p_comp.y - self.comp_offset_y
                    f.write(f"P {vis_x:.0f} {vis_y:.0f}\n")
                
                # Write edges (clipped to visible canvas)
                for edge in final_edges:
                    p1_comp, p2_comp = edge.start, edge.end
                    
                    # Convert computational coordinates to visible coordinates
                    p1_vis = Point(p1_comp.x - self.comp_offset_x, p1_comp.y - self.comp_offset_y)
                    p2_vis = Point(p2_comp.x - self.comp_offset_x, p2_comp.y - self.comp_offset_y)
                    
                    # Clip the visible-coordinate edge to the 600x600 canvas
                    clipped_segment = self._clip_to_visible_canvas(p1_vis, p2_vis)
                    
                    if clipped_segment:
                        p1_final, p2_final = clipped_segment
                        f.write(f"E {p1_final.x:.0f} {p1_final.y:.0f} {p2_final.x:.0f} {p2_final.y:.0f}\n")

            messagebox.showinfo("匯出成功", f"已成功將點和線段儲存至\n{file_path}")

        except Exception as e:
            messagebox.showerror("匯出失敗", f"寫入檔案時發生錯誤: {e}")
    def import_from_file(self):
        """
        [NEW] 從文字檔案載入點和線段並繪製它們。
        """
        file_path = filedialog.askopenfilename(
            title="選擇要匯入的圖形檔案",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not file_path:
            return

        self.clear_canvas()  # 重置應用程式狀態

        imported_points = []
        imported_edges = []

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    parts = line.strip().split()
                    if not parts:
                        continue
                    
                    # 解析點
                    if parts[0] == 'P' and len(parts) == 3:
                        x = float(parts[1])
                        y = float(parts[2])
                        imported_points.append(Point(x, y))
                    # 解析邊
                    elif parts[0] == 'E' and len(parts) == 5:
                        p1 = Point(float(parts[1]), float(parts[2]))
                        p2 = Point(float(parts[3]), float(parts[4]))
                        imported_edges.append(Edge(p1, p2))
            
            # 繪製匯入的資料
            self.draw_imported_data(imported_points, imported_edges)

        except Exception as e:
            messagebox.showerror("匯入失敗", f"讀取或解析檔案時發生錯誤: {e}")
    def draw_imported_data(self, points, edges):
        """
        [NEW] 專門用於繪製從檔案匯入的點和邊。
        """
        self.canvas.delete("all")
        
        # 先畫邊
        for edge in edges:
            p1, p2 = edge.start, edge.end
            canvas_y1 = self.visible_height - p1.y
            canvas_y2 = self.visible_height - p2.y
            self.canvas.create_line(p1.x, canvas_y1, p2.x, canvas_y2, fill="black", width=1.5)
            
        # 再畫點（這樣點會蓋在線上）
        for p in points:
            canvas_y = self.visible_height - p.y
            self.canvas.create_oval(p.x - 3, canvas_y - 3, p.x + 3, canvas_y + 3, fill="red", outline="black")
            self.canvas.create_text(p.x + 5, canvas_y - 5, text=f"({p.x:.0f}, {p.y:.0f})", anchor=tk.SW, fill="black")

    def draw_all(self, step, is_final_run=False):
        """
        [MODIFIED] Redesigned to draw everything based on a StepRecord object.
        - Renders pre-merge hulls and highlights active points.
        - Draws text first, then lines, to prevent lines from being obscured.
        """
        self.canvas.delete("all")

        # [MODIFIED] Draw points and coordinates first, placing them on the bottom layer.
        if step:
            self.draw_points(all_points=step.all_points, active_points=step.points)

        def get_drawable_edge(edge):
            if not isinstance(edge, Edge): return None
            p1_vis = Point(edge.start.x - self.comp_offset_x, edge.start.y - self.comp_offset_y)
            p2_vis = Point(edge.end.x - self.comp_offset_x, edge.end.y - self.comp_offset_y)
            return self._clip_to_visible_canvas(p1_vis, p2_vis)

        def draw_edge_list(edges, color, prefix="", width=1.5):
            for i, edge in enumerate(edges):
                drawable_pts = get_drawable_edge(edge)
                if drawable_pts:
                    p1, p2 = drawable_pts
                    y1, y2 = self.visible_height - p1.y, self.visible_height - p2.y
                    
                    if prefix and not is_final_run:
                        mid_x, mid_y = (p1.x + p2.x) / 2, (y1 + y2) / 2
                        
                        text_item = self.canvas.create_text(mid_x, mid_y, text=f"{prefix}{i}", fill="purple", font=("Arial", 8, "bold"))
                        
                        x1_b, y1_b, x2_b, y2_b = self.canvas.bbox(text_item)
                        bg_item = self.canvas.create_rectangle(x1_b-2, y1_b-1, x2_b+2, y2_b+1, fill="white", outline="")
                        
                        self.canvas.lower(bg_item, text_item)

                    self.canvas.create_line(p1.x, y1, p2.x, y2, fill=color, width=width)

        def draw_hull(hull, color, width=2.0):
             if hull and len(hull.points) > 1:
                for i in range(len(hull.points)):
                    p_start, p_end = hull.points[i], hull.points[(i + 1) % len(hull.points)]
                    drawable_pts = get_drawable_edge(Edge(p_start, p_end))
                    if drawable_pts:
                        p1, p2 = drawable_pts
                        self.canvas.create_line(p1.x, self.visible_height - p1.y, p2.x, self.visible_height - p2.y, fill=color, width=width)
        
        if not step:
            return

        # --- Drawing Execution ---
        if is_final_run:
            draw_edge_list(step.left_edges, color="black", width=1.5)
        else:
            # 在最終清理步驟，只顯示最終的黑色邊緣
            if step.description.startswith("最終清理"):
                 draw_edge_list(step.left_edges, color="black", width=1.5)
            else:
                draw_edge_list(step.left_edges, "#a6d1e6", "L", 1.5) # Light Blue
                draw_edge_list(step.right_edges, "#b5e5a4", "R", 1.5) # Light Green
                draw_edge_list(step.hyperplane, "blue", "H", 2)

        if not is_final_run:
            draw_hull(step.left_hull, "#e6f089", 6.0) # Baby Blue
            draw_hull(step.right_hull, "#e6f089", 6.0) # Light Green

        # [MODIFIED] 只有在不是最終清理步驟時，才繪製凸包
        if not step.description.startswith("最終清理"):
            draw_hull(step.convex_hull, "green", 2.0) # Final Hull


if __name__ == "__main__":
    Point.normal = lambda self: Point(-self.y, self.x)
    app = Application(width=600, height=600)
    app.mainloop()