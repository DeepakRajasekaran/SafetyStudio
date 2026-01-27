import sys
import math
import signal
import ezdxf
import numpy as np
import json
from shapely.geometry import Polygon, MultiPoint, Point, MultiPolygon, LineString, GeometryCollection, MultiLineString
from shapely import wkt
from shapely.ops import unary_union, polygonize, linemerge
from shapely.affinity import rotate, translate, scale

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsPolygonItem,
    QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsPathItem,
    QGridLayout, QTabWidget, QDialog, QRadioButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QPushButton, QLabel, 
    QLineEdit, QGroupBox, QMessageBox, QListWidget, QTextEdit, QFrame, 
    QFileDialog, QScrollArea, QToolButton, QCheckBox, QComboBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QPointF, QLineF, QRectF, QTimer
from PyQt6.QtGui import QPen, QBrush, QColor, QFont, QPainter, QPalette, QPolygonF, QFontMetrics, QPainterPath

SCALE = 100.0  # 1m = 100px

# =====================================================================
# 1. CORE LOGIC
# =====================================================================
class DxfHandler:
    @staticmethod
    def snap_lines(lines, tol=1e-3):
        # Optimized snapping using coordinate rounding (O(N)) to prevent freezing on large files
        snapped = []
        for line in lines:
            # Round to 4 decimal places (0.1mm) to fix floating point gaps
            raw_coords = [(round(x, 4), round(y, 4)) for x, y in line.coords]
            # Deduplicate consecutive points
            new_coords = [raw_coords[0]]
            for p in raw_coords[1:]:
                if p != new_coords[-1]: new_coords.append(p)
            
            if len(new_coords) >= 2:
                snapped.append(LineString(new_coords))
        return snapped

    @staticmethod
    def load(filename):
        try:
            doc = ezdxf.readfile(filename); msp = doc.modelspace()
            polys = []
            lines = []
            
            for e in msp.query('LWPOLYLINE'):
                # LWPolyline is iterable in ezdxf (yields x, y, start_width, end_width, bulge)
                # or use get_points() if available
                pts = []
                try:
                    if hasattr(e, 'get_points'): pts = e.get_points(format='xy')
                    else: pts = [(v[0], v[1]) for v in e]
                except:
                    if hasattr(e, 'vertices'): pts = [(v[0], v[1]) for v in e.vertices()]
                
                if len(pts)>=2:
                    if e.closed or (len(pts)>=3 and math.hypot(pts[0][0]-pts[-1][0], pts[0][1]-pts[-1][1]) < 1e-3):
                        if len(pts)>=3: polys.append(Polygon(pts))
                    else:
                        lines.append(LineString(pts))
            for e in msp.query('POLYLINE'):
                pts=[(float(v.dxf.location.x),float(v.dxf.location.y)) for v in e.vertices()]
                if len(pts)>=2:
                    if e.is_closed or (len(pts)>=3 and math.hypot(pts[0][0]-pts[-1][0], pts[0][1]-pts[-1][1]) < 1e-3):
                        if len(pts)>=3: polys.append(Polygon(pts))
                    else:
                        lines.append(LineString(pts))
            for e in msp.query('LINE'):
                p_s, p_e = (e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)
                lines.append(LineString([p_s, p_e]))
            
            # Support for ARCs (discretize)
            for e in msp.query('ARC'):
                c = e.dxf.center; r = e.dxf.radius
                sa = e.dxf.start_angle; ea = e.dxf.end_angle
                if ea < sa: ea += 360
                # Discretize every 5 degrees
                angles = np.linspace(np.radians(sa), np.radians(ea), max(2, int(abs(ea-sa)/5)+1))
                arc_pts = [(c.x + r*np.cos(a), c.y + r*np.sin(a)) for a in angles]
                lines.append(LineString(arc_pts))
            
            # Try to infer polygons from lines (for "exploded" CAD)
            if lines:
                # 0. Snap lines to fix gaps
                lines = DxfHandler.snap_lines(lines)
                
                # 1. Merge touching lines to form longer chains/loops
                merged = linemerge(lines)
                if merged.geom_type == 'LineString': merged_lines = [merged]
                elif merged.geom_type == 'MultiLineString': merged_lines = list(merged.geoms)
                else: merged_lines = []
                
                # 2. Polygonize the merged lines
                for p in polygonize(merged_lines): polys.append(p)
                
                # 3. Keep lines that didn't form polygons (optional, but good for debugging/incomplete shapes)
                polys.extend(merged_lines)
            
            if not polys: raise Exception("No geometry found in DXF.")
            return unary_union(polys)
        except Exception as e:
            raise Exception(f"{str(e)}")

class SafetyMath:
    @staticmethod
    def get_shadow_wedge(og, load_poly, r):
        if not load_poly or load_poly.is_empty: return None
        try:
            # 1. Decompose Load Geometry (Per-Component Shadowing)
            geoms = load_poly.geoms if hasattr(load_poly, 'geoms') else [load_poly]
            wedges = []
            
            for g in geoms:
                pts = []
                if g.geom_type == 'Polygon': pts = list(g.exterior.coords)
                elif g.geom_type in ['LineString', 'LinearRing']: pts = list(g.coords)
                if not pts: continue
                
                # 2. Shadow Computation Per Component
                angles = sorted([np.arctan2(y - og[1], x - og[0]) for x, y in pts])
                if not angles: continue
                
                max_gap = 0; gap_idx = -1; n = len(angles)
                for i in range(n):
                    diff = (angles[(i+1)%n] + 2*np.pi - angles[i]) if i==n-1 else (angles[i+1] - angles[i])
                    if diff > max_gap: max_gap = diff; gap_idx = i
                
                start = angles[(gap_idx+1)%n]; end = angles[gap_idx]
                if end < start: end += 2*np.pi
                
                # 3. Compute Near-Field Cutoff (Min Distance)
                # Use Shapely distance from Point(og) to the geometry g
                min_dist = Point(og).distance(g)
                
                # If object is further than max range, it casts no relevant shadow
                if min_dist >= r: continue
                
                # 4. Build Truncated Shadow Wedge (Annular Sector)
                # Inner arc at min_dist, Outer arc at r
                wedge_pts = []
                steps = max(2, int(np.degrees(end-start)/2.0)+2)
                
                # Inner Arc (CCW)
                if min_dist < 1e-3: wedge_pts.append(og)
                else:
                    for a in np.linspace(start, end, steps):
                        wedge_pts.append((og[0]+min_dist*np.cos(a), og[1]+min_dist*np.sin(a)))
                
                # Outer Arc (CW)
                for a in np.linspace(end, start, steps):
                    wedge_pts.append((og[0]+r*np.cos(a), og[1]+r*np.sin(a)))
                
                wedges.append(Polygon(wedge_pts))
            
            # 3. Union Shadows Conservatively
            if not wedges: return None
            return unary_union(wedges)
        except: return None

    @staticmethod
    def patch_notch(poly):
        """
        Post-processing repair for in-place rotation fields.
        1. Removes noisy concave points from the outer arc.
        2. Bridges notches (radius spikes) in the inner arc with a fitted arc.
        """
        if not poly.is_valid or poly.is_empty or poly.geom_type != 'Polygon': return poly
        
        # Ensure CCW
        if not poly.exterior.is_ccw:
            poly = Polygon(list(poly.exterior.coords)[::-1])
            
        pts = list(poly.exterior.coords)
        if pts[0] == pts[-1]: pts.pop()
        n = len(pts)
        if n < 10: return poly
        
        # Metrics
        radii = np.array([math.hypot(p[0], p[1]) for p in pts])
        max_r = np.max(radii)
        min_r = np.min(radii)
        
        # --- PATCH 1: Top/Outer Arc Cleanup ---
        # Remove sharp concave dents (Right Turns) on the outer boundary (High Radius)
        keep_indices = []
        for i in range(n):
            p_prev = pts[(i-1)%n]; p_curr = pts[i]; p_next = pts[(i+1)%n]
            # Cross product (z-comp): Positive=Left(Convex), Negative=Right(Concave)
            cp = (p_curr[0]-p_prev[0])*(p_next[1]-p_curr[1]) - (p_curr[1]-p_prev[1])*(p_next[0]-p_curr[0])
            
            is_outer = radii[i] > (max_r * 0.85)
            is_dent = cp < -1e-4 
            
            # If it's a dent on the outer shell, skip it (remove)
            if is_outer and is_dent: continue
            keep_indices.append(i)
            
        pts = [pts[i] for i in keep_indices]
        n = len(pts)
        if n < 5: return Polygon(pts)
        radii = np.array([math.hypot(p[0], p[1]) for p in pts]) # Recompute
        
        # --- PATCH 2: Bottom/Inner Arc Notch Repair ---
        # Detect "Notch" in Inner region (Low Radius).
        # Notch = Region where radius spikes UP (indentation into the solid).
        
        # 1. Rotate to start at Max Radius (Outer) to safely isolate Inner block
        max_idx = np.argmax(radii)
        pts = pts[max_idx:] + pts[:max_idx]
        radii = np.roll(radii, -max_idx)
        
        # 2. Identify Inner Segment (Low Radius)
        thresh = min_r + (max_r - min_r) * 0.4
        inner_indices = [i for i, r in enumerate(radii) if r < thresh]
        
        if not inner_indices: return Polygon(pts)
        
        # Find the largest contiguous block of inner indices
        segments = []; curr = [inner_indices[0]]
        for x in inner_indices[1:]:
            if x == curr[-1] + 1: curr.append(x)
            else: segments.append(curr); curr = [x]
        segments.append(curr)
        inner_seg = max(segments, key=len)
        
        # 3. Detect Notch within Inner Segment (Radius Spike)
        seg_radii = radii[inner_seg]
        base_r = np.min(seg_radii)
        notch_thresh = base_r * 1.2 # >20% deviation
        
        bad_sub = [i for i in range(len(inner_seg)) if seg_radii[i] > notch_thresh]
        
        # Only patch if notch is internal (surrounded by valid inner arc points)
        if bad_sub and bad_sub[0] > 0 and bad_sub[-1] < len(inner_seg) - 1:
            # Define Notch Bounds
            first_bad = inner_seg[bad_sub[0]]
            last_bad = inner_seg[bad_sub[-1]]
            idx_start = first_bad - 1
            idx_end = last_bad + 1
            
            if idx_start >= 0 and idx_end < len(pts):
                p_s = pts[idx_start]; p_e = pts[idx_end]
                
                # 4. Interpolate Arc (Fit to start/end)
                # Simple fit: Center at (0,0) for in-place rotation is physically robust
                # and matches "implicit arc" of a rotation.
                r_fit = (math.hypot(p_s[0], p_s[1]) + math.hypot(p_e[0], p_e[1])) / 2.0
                a_s = math.atan2(p_s[1], p_s[0]); a_e = math.atan2(p_e[1], p_e[0])
                
                diff = a_e - a_s
                if diff < -math.pi: diff += 2*math.pi
                if diff > math.pi: diff -= 2*math.pi
                
                steps = int(abs(diff) * r_fit / 0.05) + 1
                new_arc = []
                for s in range(steps + 1):
                    t = s / steps; ang = a_s + diff * t
                    new_arc.append((r_fit*math.cos(ang), r_fit*math.sin(ang)))
                
                pts = pts[:idx_start] + new_arc + pts[idx_end+1:]

        return Polygon(pts)

    @staticmethod
    def calc_case(footprint, load_poly, sensors, v, w_input, P):
        try:
            # 1. Geometry Prep (True Footprint vs Padded)
            raw_footprint_poly = footprint
            
            # Apply Lateral Scaling to Raw Footprint (before padding)
            lat_scale = P.get('lat_scale', 1.0)
            if lat_scale != 1.0:
                raw_footprint_poly = scale(raw_footprint_poly, xfact=lat_scale, yfact=1.0, origin=(0,0))
            
            # Define True Geometry (Unpadded) for Dynamics
            if load_poly:
                # Union with load (unpadded) to get full rigid body
                unpadded_geom = unary_union([raw_footprint_poly, load_poly]).convex_hull
            else:
                unpadded_geom = raw_footprint_poly

            # Define Padded Geometry (For Sweeping)
            FootPrint = unpadded_geom.buffer(P['pad'], join_style=2)
            
            # 2. Dynamics (Model-Independent)
            # w_input is authoritative angular velocity (rad/s)
            ang_vel = w_input
            
            # 3. Identify Canonical Reference Point (ref_pt)
            # F = unpadded_geom
            if hasattr(unpadded_geom, 'geoms'):
                pts = []
                for g in unpadded_geom.geoms: 
                    if hasattr(g, 'exterior'): pts.extend(list(g.exterior.coords))
                    elif hasattr(g, 'coords'): pts.extend(list(g.coords))
            else:
                if hasattr(unpadded_geom, 'exterior'): pts = list(unpadded_geom.exterior.coords)
                elif hasattr(unpadded_geom, 'coords'): pts = list(unpadded_geom.coords)
                else: pts = []
            if not pts: pts = [(0,0)]
            # Deterministic sort for tie-breaking
            pts.sort(key=lambda p: (p[0], p[1]))
            
            # Reference Point Selection
            EPS_W = 1e-5
            if abs(ang_vel) < EPS_W:
                # Linear Case (w=0): Select leading edge based on v direction (Y-forward)
                ref_pt = max(pts, key=lambda p: p[1]) if v >= -1e-9 else min(pts, key=lambda p: p[1])
                v_ref = abs(v)
            else:
                # Angular Case (w!=0):
                # Rule: Use Opposite/Inner Edge for Safety Arc Reference.
                # w > 0 (Left Turn) -> Inner is Left (-X)
                # w < 0 (Right Turn) -> Inner is Right (+X)
                
                sign_w = 1.0 if ang_vel >= 0 else -1.0
                # Use effective linear velocity direction for front/back selection
                v_eff = v if abs(v) > 1e-6 else 1e-6
                sign_v = 1.0 if v_eff >= 0 else -1.0
                
                def score(p):
                    # Primary: Lateral Side (Opposite to turn)
                    lat_score = -sign_w * p[0] * 1000.0
                    # Secondary: Longitudinal (Leading in motion)
                    long_score = sign_v * p[1]
                    return lat_score + long_score
                
                ref_pt = max(pts, key=score)
                
                # Physics speed
                vx_real = -ang_vel * ref_pt[1]
                vy_real = v + ang_vel * ref_pt[0]
                v_ref = math.sqrt(vx_real**2 + vy_real**2)
            
            # 4. Stop Distance & Time (Based on Reference Point Speed)
            D = v_ref * P['tr'] + (v_ref**2) / (2 * P['ac']) + P['ds'] if P['ac'] > 0 else v_ref * P['tr'] + P['ds']
            T = D / v_ref if v_ref > 0.01 else P['tr']
            
            # 5. Sweep (Standard Stepwise for all cases)
            dt=0.02; ts=np.arange(0,T+dt,dt); sweeps=[]
            traj=np.zeros((len(ts),3)); traj[0]=[0,0,np.pi/2] # Start Y+
            
            for i in range(len(ts)):
                if i>0:
                    px,py,pth = traj[i-1]
                    traj[i] = [px+v*np.cos(pth)*dt, py+v*np.sin(pth)*dt, pth+ang_vel*dt]
                
                cx,cy,cth = traj[i]; rot_deg = np.degrees(cth - np.pi/2)
                poly_instance = translate(rotate(FootPrint, rot_deg, origin=(0,0)), cx, cy)
                sweeps.append(poly_instance)
            
            # 5. Union
            sw_union = unary_union(sweeps)
            
            # 5b. Post-Processing Patch for In-Place Rotation (V-Notch Fix)
            if abs(v) < 1e-3 and abs(ang_vel) > 1e-3 and P.get('patch_notch', False):
                sw_union = SafetyMath.patch_notch(sw_union)
            
            # Add smoothing
            final = sw_union.buffer(P.get('smooth',0.05), join_style=1).simplify(0.01)
            
            # 6. Sensor Cuts
            lid_out = []
            all_fovs = []
            composite_clips = []
            th0 = np.pi/2 
            for s in sensors:
                og=(s['x'], s['y']); max_r = s['r']
                mid=np.radians(90+s['mount']); hw=np.radians(s['fov']/2)
                
                # Create exact sensor wedge
                FootPrint_pts = [(og[0]+max_r*np.cos(a), og[1]+max_r*np.sin(a)) for a in np.linspace(mid-hw,mid+hw,50)]
                fov = Polygon([og]+FootPrint_pts+[og])
                all_fovs.append(fov)
                
                clip = final.intersection(fov)
                
                if load_poly:
                    clip = clip.difference(load_poly)
                
                # Store unshadowed clip for composite view
                composite_clips.append(clip)
                
                # Shadowing (Only for individual lidar fields)
                shadow = None
                clip_indiv = clip
                if load_poly and P.get('shadow', True):
                    shadow = SafetyMath.get_shadow_wedge(og, load_poly, max_r*1.1)
                    if shadow: clip_indiv = clip.difference(shadow)

                # To Local
                s_rot=np.radians(90+s['mount']); loc=[]
                if not clip_indiv.is_empty:
                    c,si=np.cos(-s_rot),np.sin(-s_rot); R=np.array([[c,-si],[si,c]])
                    gs=clip_indiv.geoms if clip_indiv.geom_type=='MultiPolygon' else [clip_indiv]
                    for g in gs: loc.append( (np.array(g.exterior.coords)-np.array(og)) @ R.T )
                
                lid_out.append({'name':s['name'], 'clip':clip_indiv, 'origin':og, 'local':loc, 'fov_poly':fov, 'shadow_poly':shadow, 'mount':s['mount']})
                
            # Calculate Ignored Region (Kinematic Field outside Sensor FOVs)
            ignored_poly = None
            if all_fovs:
                # Ignored = Raw Field - Union(FOVs) - Load
                u_fov = unary_union(all_fovs)
                ignored_poly = final.difference(u_fov)
                if load_poly: ignored_poly = ignored_poly.difference(load_poly)

            # Update Global Field to be Effective Field (Union of Sensors)
            if composite_clips:
                final = unary_union(composite_clips)
            elif load_poly:
                final = final.difference(load_poly)
            
            # Front Trajectory (Visualization of Reference Point)
            front_traj = []
            for i in range(len(traj)):
                cx, cy, cth = traj[i]
                rot_deg = np.degrees(cth - np.pi/2)
                # Transform ref_pt to global frame
                pt_global = translate(rotate(Point(ref_pt), rot_deg, origin=(0,0)), cx, cy)
                front_traj.append((pt_global.x, pt_global.y))
            
            return final, lid_out, traj, sweeps, D, front_traj, ignored_poly
        except Exception as e:
            import traceback
            traceback.print_exc()
            return None, str(e), [], [], 0.0, [], None

# =====================================================================
# 2. SHARED VISUALIZATION
# =====================================================================
class BaseGridScene(QGraphicsScene):
    def __init__(self):
        super().__init__(); self.setSceneRect(-3000,-3000,6000,6000)
        
    def drawBackground(self, p, r):
        p.fillRect(r, QColor("#1c1c1c"))
        step = int(1.0 * SCALE)
        l,t,ri,b = int(r.left()), int(r.top()), int(r.right()), int(r.bottom())
        
        p.setPen(QPen(QColor("#333333"), 0))
        fx = l - (l % step)
        for x in range(fx, ri, step): p.drawLine(x, t, x, b)
        fy = t - (t % step)
        for y in range(fy, b, step): p.drawLine(l, y, ri, y)
        
        p.setPen(QPen(QColor("#D32F2F"), 3)); p.drawLine(0,0, 100,0)   
        p.setPen(QPen(QColor("#388E3C"), 3)); p.drawLine(0,0, 0,-100) 
        p.setBrush(QBrush(QColor("white"))); p.setPen(Qt.PenStyle.NoPen); p.drawEllipse(-3,-3,6,6)

class RulerView(QGraphicsView):
    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.r_bg=QColor("#2d2d2d"); self.r_fg=QColor("#666"); self.r_txt=QColor("#ccc"); self.margin=30

    def drawForeground(self, p, rect):
        p.resetTransform(); w, h = self.viewport().width(), self.viewport().height()
        p.fillRect(0,0, w, self.margin, self.r_bg); p.fillRect(0,0, self.margin, h, self.r_bg)
        p.setPen(QColor("#444")); p.drawLine(0, self.margin, w, self.margin); p.drawLine(self.margin, 0, self.margin, h)
        
        m_px = 1.0 / (self.transform().m11() * SCALE)
        raw = 80 * m_px; base = 10**math.floor(math.log10(raw) if raw>0 else 0)
        step_m = (1 if raw/base < 2 else (2 if raw/base < 5 else 5)) * base
        step_px = step_m * SCALE
        
        v = self.mapToScene(self.viewport().rect()).boundingRect()
        
        # X
        l_m = v.left(); start_x = math.floor(l_m / step_px) * step_px
        x = start_x
        while x < v.right():
            scr = self.mapFromScene(x, 0).x()
            if self.margin < scr < w:
                p.setPen(self.r_fg); p.drawLine(int(scr), 15, int(scr), 30)
                txt = f"{x/SCALE:.2g}"; fm=QFontMetrics(p.font()); tw=fm.horizontalAdvance(txt)
                p.setPen(self.r_txt); p.drawText(int(scr-tw/2), 12, txt)
            x += step_px
            
        # Y
        t_m = v.top(); start_y = math.floor(t_m / step_px) * step_px
        y = start_y
        while y < v.bottom():
            scr = self.mapFromScene(0, y).y()
            if self.margin < scr < h:
                p.setPen(self.r_fg); p.drawLine(15, int(scr), 30, int(scr))
                txt = f"{-y/SCALE:.2g}"; p.save(); p.translate(12, scr); p.rotate(-90)
                p.setPen(self.r_txt); p.drawText(int(-p.fontMetrics().horizontalAdvance(txt)/2), 0, txt); p.restore()
            y += step_px
        p.setPen(QColor("#FF1744")); p.drawText(self.margin+5, 20, "X")
        p.setPen(QColor("#00C853")); p.drawText(8, 45, "Y")

    def wheelEvent(self, e):
        s = 1.1 if e.angleDelta().y() > 0 else 0.9; self.scale(s,s)

# =====================================================================
# 3. EDITOR COMPONENTS
# =====================================================================
class EditorScene(BaseGridScene):
    def __init__(self):
        super().__init__()
        self.ditems={'FootPrint':[],'L1':[],'L2':[]}; self.sitems=[]; self.vflags={'FootPrint':1,'L1':1,'L2':1}
    def update_polys(self, FootPrint, L1, L2):
        for k in ['FootPrint','L1','L2']: [self.removeItem(i) for i in self.ditems[k]]; self.ditems[k]=[]
        def add(p, c, k):
            if not p or not self.vflags[k]: return
            # Handle legacy list-of-points data if present
            if isinstance(p, list):
                try: p = Polygon(p)
                except: return
            
            geoms = p.geoms if hasattr(p, 'geoms') else [p]
            for g in geoms:
                if g.geom_type == 'Polygon':
                    pts=[QPointF(x*SCALE,-y*SCALE) for x,y in g.exterior.coords]
                    item=QGraphicsPolygonItem(QPolygonF(pts)); item.setBrush(QBrush(QColor(c))); item.setOpacity(0.4)
                    item.setPen(QPen(Qt.PenStyle.NoPen)); self.addItem(item); self.ditems[k].append(item)
                elif g.geom_type in ['LineString', 'LinearRing']:
                    pts=[QPointF(x*SCALE,-y*SCALE) for x,y in g.coords]
                    path = QPainterPath()
                    path.moveTo(pts[0])
                    for pt in pts[1:]: path.lineTo(pt)
                    item = QGraphicsPathItem(path)
                    item.setPen(QPen(QColor(c), 2)); item.setOpacity(0.6)
                    self.addItem(item); self.ditems[k].append(item)
        add(FootPrint,'#999','FootPrint'); add(L1,'#2196F3','L1'); add(L2,'#4CAF50','L2')
    def update_sensors(self, s_data):
        [self.removeItem(i) for i in self.sitems]; self.sitems=[]
        for s in s_data:
            g=QGraphicsEllipseItem(-8,-8,16,16); g.setPos(s['x']*SCALE, -s['y']*SCALE)
            g.setBrush(QBrush(QColor("magenta"))); g.setPen(QPen(Qt.GlobalColor.white,1))
            self.addItem(g); self.sitems.append(g)
            l=QGraphicsLineItem(0,0,0,-20,g); l.setPen(QPen(Qt.GlobalColor.white,2)); g.setRotation(-s['mount'])

class EditorTab(QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main; l = QHBoxLayout(self)
        
        # Left Scene
        lc=QWidget(); lv=QVBoxLayout(lc); lv.setContentsMargins(0,0,0,0)
        tb=QHBoxLayout(); tb.addWidget(QLabel("Show:"))
        for k in ['FootPrint','L1','L2']:
            c=QCheckBox(k); c.setChecked(True); c.toggled.connect(lambda v,x=k: self.tog(x,v)); tb.addWidget(c)
        tb.addStretch(); lv.addLayout(tb)
        self.scn=EditorScene(); self.view=RulerView(self.scn)
        lv.addWidget(self.view); l.addWidget(lc, 2)
        
        # Right Config
        r=QFrame(); r.setFixedWidth(300); rv=QVBoxLayout(r)
        rv.addWidget(QLabel("<b>LiDAR Manager</b>"))
        self.lst=QListWidget(); self.lst.setFixedHeight(120); self.lst.currentRowChanged.connect(self.ld)
        rv.addWidget(self.lst)
        bb=QHBoxLayout(); ba=QPushButton("+"); bd=QPushButton("-"); bb.addWidget(ba); bb.addWidget(bd)
        ba.clicked.connect(self.add); bd.clicked.connect(self.dele); rv.addLayout(bb)
        
        fm=QGridLayout()
        self.i_nm=QLineEdit(); self.i_x=QLineEdit(); self.i_y=QLineEdit()
        self.i_mn=QLineEdit(); self.i_fv=QLineEdit(); self.i_rg=QLineEdit() # ADDED RANGE INPUT
        fm.addWidget(QLabel("Nm"),0,0); fm.addWidget(self.i_nm,0,1)
        fm.addWidget(QLabel("X"),1,0); fm.addWidget(self.i_x,1,1); fm.addWidget(QLabel("Y"),1,2); fm.addWidget(self.i_y,1,3)
        fm.addWidget(QLabel("Deg"),2,0); fm.addWidget(self.i_mn,2,1); fm.addWidget(QLabel("FOV"),2,2); fm.addWidget(self.i_fv,2,3)
        fm.addWidget(QLabel("Rng"),3,0); fm.addWidget(self.i_rg,3,1)
        
        bu=QPushButton("Apply Sensor"); bu.clicked.connect(self.sav); rv.addLayout(fm); rv.addWidget(bu); rv.addStretch()
        
        rv.addWidget(QFrame(frameShape=QFrame.Shape.HLine))
        rv.addWidget(QLabel("<b>Geometry Loader</b>"))
        for k in ['FootPrint','L1','L2']:
            hb=QHBoxLayout()
            b=QPushButton(f"Load {k}"); b.clicked.connect(lambda _,x=k: self.load_sh(x))
            b.setStyleSheet("background:#444;color:white;padding:4px"); hb.addWidget(b)
            c=QPushButton("Clr"); c.setFixedWidth(40); c.setStyleSheet("background:#B71C1C;color:white;padding:4px"); c.clicked.connect(lambda _,x=k: self.clear_sh(x))
            hb.addWidget(c); rv.addLayout(hb)
        rv.addStretch(); l.addWidget(r)
        
        self.polys={'FootPrint':[],'L1':[],'L2':[]}
        self.sens=[{'name':'Main','x':0.45,'y':0,'mount':0,'fov':270,'r':10.0}]
        self.upl(); QTimer.singleShot(100, lambda: self.view.centerOn(0,0))
    
    def load_sh(self, k): 
        f,_=QFileDialog.getOpenFileName(self,"","","DXF (*.dxf)");
        if f:
            try:
                p=DxfHandler.load(f)
                self.polys[k]=p; self.render()
            except Exception as e:
                QMessageBox.critical(self, "Import Failed", f"Could not load DXF file:\n{f}\n\nError Details:\n{str(e)}\n\nEnsure the file contains valid geometry (Lines, Polylines, Arcs).")
    def clear_sh(self, k): self.polys[k]=[]; self.render()
    def render(self): self.scn.update_polys(self.polys['FootPrint'], self.polys['L1'], self.polys['L2'])
    def tog(self,k,v): self.scn.vflags[k]=v; self.render()
    def upl(self): 
        self.lst.clear(); [self.lst.addItem(s['name']) for s in self.sens]; self.scn.update_sensors(self.sens)
    def ld(self, r): 
        if r>=0: d=self.sens[r]; self.i_nm.setText(d['name']); self.i_x.setText(str(d['x'])); self.i_y.setText(str(d['y'])); self.i_mn.setText(str(d['mount'])); self.i_fv.setText(str(d['fov'])); self.i_rg.setText(str(d['r']))
    def sav(self):
        r=self.lst.currentRow()
        if r>=0: self.sens[r]={'name':self.i_nm.text(),'x':float(self.i_x.text()),'y':float(self.i_y.text()),'mount':float(self.i_mn.text()),'fov':float(self.i_fv.text()),'r':float(self.i_rg.text())}; self.upl()
    def add(self): self.sens.append({'name':'New','x':0,'y':0,'mount':0,'fov':270,'r':10.0}); self.upl()
    def dele(self):
        if self.lst.currentRow() >= 0:
            del self.sens[self.lst.currentRow()]
        self.upl()
    def get_s(self): return self.sens

# =====================================================================
# 5. TAB 2: GENERATOR (Logic)
# =====================================================================
class GenTab(QWidget):
    def __init__(self, main):
        super().__init__(); self.main=main; l=QHBoxLayout(self)
        
        # Inputs
        f=QFrame(); f.setFixedWidth(500); fl=QVBoxLayout(f)
        gp=QGroupBox("Physics Config"); gl=QGridLayout(gp)
        
        eq=QLabel("D = v*Tr + v^2/2a + Ds"); eq.setStyleSheet("color:cyan; font-weight:bold; margin-bottom:5px")
        gl.addWidget(eq,0,0,1,2)
        
        self.ptr=self.mk(gl,"Tr (Reaction)",0.3,1); self.pac=self.mk(gl,"a (Decel)",1.0,2)
        self.pds=self.mk(gl,"Ds (Buffer)",0.15,3); self.ppd=self.mk(gl,"Pad (Static)",0.1,4)
        self.psm=self.mk(gl,"Sm (Smooth)",0.05,5); 
        # Scale %
        self.lat_s=self.mk(gl,"Scale (Width%)",1.0,6)
        fl.addWidget(gp)
        
        gi=QGroupBox("Plan Auto-Gen"); gl2=QGridLayout(gi)
        
        gl2.addWidget(QLabel("Type"),0,0); gl2.addWidget(QLabel("Count"),0,1)
        gl2.addWidget(QLabel("Max V"),0,2); gl2.addWidget(QLabel("Max W"),0,3); gl2.addWidget(QLabel("Shadow"),0,4)
        
        self.chk_nl=QCheckBox("NoLoad"); self.chk_nl.setChecked(True)
        self.cnt_nl=QLineEdit("6"); self.v_nl=QLineEdit("1.2"); self.w_nl=QLineEdit("30.0")
        gl2.addWidget(self.chk_nl,1,0); gl2.addWidget(self.cnt_nl,1,1); gl2.addWidget(self.v_nl,1,2); gl2.addWidget(self.w_nl,1,3)
        
        self.chk_l1=QCheckBox("Load1"); self.chk_l1.setChecked(False)
        self.cnt_l1=QLineEdit("6"); self.v_l1=QLineEdit("1.0"); self.w_l1=QLineEdit("20.0"); self.sh_l1=QCheckBox(); self.sh_l1.setChecked(True)
        gl2.addWidget(self.chk_l1,2,0); gl2.addWidget(self.cnt_l1,2,1); gl2.addWidget(self.v_l1,2,2); gl2.addWidget(self.w_l1,2,3); gl2.addWidget(self.sh_l1,2,4)
        
        self.chk_l2=QCheckBox("Load2"); self.chk_l2.setChecked(False)
        self.cnt_l2=QLineEdit("6"); self.v_l2=QLineEdit("0.8"); self.w_l2=QLineEdit("15.0"); self.sh_l2=QCheckBox(); self.sh_l2.setChecked(True)
        gl2.addWidget(self.chk_l2,3,0); gl2.addWidget(self.cnt_l2,3,1); gl2.addWidget(self.v_l2,3,2); gl2.addWidget(self.w_l2,3,3); gl2.addWidget(self.sh_l2,3,4)
        
        self.c_fwd=QCheckBox("Fwd Linear"); self.c_fwd.setChecked(True)
        self.c_ip=QCheckBox("In-Place Spin"); self.c_ip.setChecked(True)
        self.c_turn=QCheckBox("Curve Turn"); self.c_turn.setChecked(True)
        self.c_notch=QCheckBox("Patch Notches"); self.c_notch.setChecked(True)
        
        hb=QHBoxLayout(); hb.addWidget(self.c_fwd); hb.addWidget(self.c_ip); hb.addWidget(self.c_turn); hb.addWidget(self.c_notch)
        gl2.addLayout(hb,4,0,1,5)

        bg=QPushButton("Populate Table"); bg.clicked.connect(self.pop)
        gl2.addWidget(bg,5,0,1,5); fl.addWidget(gi)
        
        gc=QHBoxLayout(); be=QPushButton("Export Config"); bi=QPushButton("Import Config")
        be.clicked.connect(self.exp_conf); bi.clicked.connect(self.imp_conf)
        gc.addWidget(be); gc.addWidget(bi)
        fl.addLayout(gc)
        
        xb=QPushButton("EXECUTE BATCH"); xb.setStyleSheet("background:#2E7D32;color:white;font-weight:bold;height:45px")
        xb.clicked.connect(self.exe); fl.addWidget(xb); fl.addStretch(); l.addWidget(f)
        
        # Table
        self.tbl=QTableWidget(0,4); self.tbl.setHorizontalHeaderLabels(["ID","Load","Vel","W(deg)"])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        l.addWidget(self.tbl)
    
    def mk(self,l,t,v,r): l.addWidget(QLabel(t),r,0); e=QLineEdit(str(v)); l.addWidget(e,r,1); return e
    
    def pop(self):
        try:
            self.tbl.setRowCount(0); idx=1
            
            cfgs = []
            if self.chk_nl.isChecked(): cfgs.append(('NoLoad', int(self.cnt_nl.text()), float(self.v_nl.text()), float(self.w_nl.text())))
            if self.chk_l1.isChecked(): cfgs.append(('Load1', int(self.cnt_l1.text()), float(self.v_l1.text()), float(self.w_l1.text())))
            if self.chk_l2.isChecked(): cfgs.append(('Load2', int(self.cnt_l2.text()), float(self.v_l2.text()), float(self.w_l2.text())))
            
            use_lin = self.c_fwd.isChecked()
            use_ip = self.c_ip.isChecked()
            use_crv = self.c_turn.isChecked()
            
            for ld, cnt, mv, mw in cfgs:
                rem = cnt
                
                # In-Place (Velocity 0)
                if use_ip and rem > 0:
                     self.ar(idx, ld, 0, mw); idx+=1; rem-=1
                     if rem > 0:
                        self.ar(idx, ld, 0, -mw); idx+=1; rem-=1
                
                if rem <= 0: continue
                
                modes = (1 if use_lin else 0) + (2 if use_crv else 0)
                if modes == 0: continue
                
                steps = math.ceil(rem / modes)
                v_step = mv / steps if steps > 0 else mv
                
                curr_v = v_step
                for _ in range(steps):
                    if use_lin and rem > 0: self.ar(idx, ld, curr_v, 0); idx+=1; rem-=1
                    if use_crv:
                        if rem > 0: self.ar(idx, ld, curr_v, mw); idx+=1; rem-=1
                        if rem > 0: self.ar(idx, ld, curr_v, -mw); idx+=1; rem-=1
                    curr_v += v_step

        except Exception as e: print(e)

    def ar(self,i,l,v,w):
        r=self.tbl.rowCount(); self.tbl.insertRow(r)
        self.tbl.setItem(r,0,QTableWidgetItem(str(i)))
        self.tbl.setItem(r,1,QTableWidgetItem(l))
        self.tbl.setItem(r,2,QTableWidgetItem(f"{v:.2f}"))
        self.tbl.setItem(r,3,QTableWidgetItem(f"{w:.1f}"))
    
    def exe(self): self.main.run_sim()
    
    def exp_conf(self):
        # Serialize geometry to WKT (or keep as is if None)
        geo_data = {}
        for k, v in self.main.ed.polys.items():
            if v is not None and not v.is_empty:
                geo_data[k] = v.wkt

        d={
            'phy':{'tr':self.ptr.text(),'ac':self.pac.text(),'ds':self.pds.text(),'pad':self.ppd.text(),'sm':self.psm.text(),'ls':self.lat_s.text()},
            'bat':{
                'nl':{'en':self.chk_nl.isChecked(),'c':self.cnt_nl.text(),'v':self.v_nl.text(),'w':self.w_nl.text()},
                'l1':{'en':self.chk_l1.isChecked(),'c':self.cnt_l1.text(),'v':self.v_l1.text(),'w':self.w_l1.text(),'sh':self.sh_l1.isChecked()},
                'l2':{'en':self.chk_l2.isChecked(),'c':self.cnt_l2.text(),'v':self.v_l2.text(),'w':self.w_l2.text(),'sh':self.sh_l2.isChecked()},
                'fwd':self.c_fwd.isChecked(),'ip':self.c_ip.isChecked(),'tn':self.c_turn.isChecked(),'pn':self.c_notch.isChecked()
            },
            'sns':self.main.ed.get_s(),
            'geo':geo_data
        }
        f,_=QFileDialog.getSaveFileName(self,"Save","","JSON (*.json)")
        if f: 
            with open(f,'w') as fp: json.dump(d,fp,indent=2)
            
    def imp_conf(self):
        f,_=QFileDialog.getOpenFileName(self,"Open","","JSON (*.json)")
        if f:
            try:
                with open(f,'r') as fp: d=json.load(fp)
                p=d.get('phy',{})
                if 'tr' in p: self.ptr.setText(p['tr'])
                if 'ac' in p: self.pac.setText(p['ac'])
                if 'ds' in p: self.pds.setText(p['ds'])
                if 'pad' in p: self.ppd.setText(p['pad'])
                if 'sm' in p: self.psm.setText(p['sm'])
                if 'ls' in p: self.lat_s.setText(p['ls'])
                b=d.get('bat',{})
                if 'nl' in b:
                    v=b['nl']
                    if isinstance(v, dict): self.chk_nl.setChecked(v.get('en',True)); self.cnt_nl.setText(str(v.get('c','6'))); self.v_nl.setText(str(v.get('v','1.2'))); self.w_nl.setText(str(v.get('w','30.0')))
                    else: self.chk_nl.setChecked(bool(v))
                
                v=b.get('l1', b.get('L1'))
                if v is not None:
                    if isinstance(v, dict): self.chk_l1.setChecked(v.get('en',False)); self.cnt_l1.setText(str(v.get('c','6'))); self.v_l1.setText(str(v.get('v','1.0'))); self.w_l1.setText(str(v.get('w','20.0'))); self.sh_l1.setChecked(v.get('sh',True))
                    else: self.chk_l1.setChecked(bool(v))
                
                v=b.get('l2', b.get('L2'))
                if v is not None:
                    if isinstance(v, dict): self.chk_l2.setChecked(v.get('en',False)); self.cnt_l2.setText(str(v.get('c','6'))); self.v_l2.setText(str(v.get('v','0.8'))); self.w_l2.setText(str(v.get('w','15.0'))); self.sh_l2.setChecked(v.get('sh',True))
                    else: self.chk_l2.setChecked(bool(v))
                
                if 'fwd' in b: self.c_fwd.setChecked(b['fwd'])
                if 'ip' in b: self.c_ip.setChecked(b['ip'])
                if 'tn' in b: self.c_turn.setChecked(b['tn'])
                if 'pn' in b: self.c_notch.setChecked(b['pn'])
                s=d.get('sns',[])
                if s: self.main.ed.sens=s; self.main.ed.upl()
                g=d.get('geo',{})
                if g: 
                    # Handle both legacy (list of points) and new (WKT string) formats
                    for k, v in g.items():
                        if isinstance(v, list): # Legacy
                            try: self.main.ed.polys[k] = Polygon(v)
                            except: pass
                        elif isinstance(v, str): # WKT
                            try: self.main.ed.polys[k] = wkt.loads(v)
                            except: pass
                    self.main.ed.render()
            except Exception as e: QMessageBox.critical(self,"Err",str(e))

# =====================================================================
# 6. TAB 3: RESULTS
# =====================================================================
class EditHandle(QGraphicsEllipseItem):
    def __init__(self, x, y, poly_item, idx, cb):
        super().__init__(-3,-3,6,6)
        self.setPos(x,y); self.pi=poly_item; self.idx=idx; self.cb=cb
        self.setBrush(QBrush(QColor("#D50000"))); self.setPen(QPen(Qt.GlobalColor.white,1))
        self.setFlags(QGraphicsItem.GraphicsItemFlag.ItemIsMovable|QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.setZValue(100)
    def itemChange(self, c, v):
        if c==QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            p=self.pi.polygon(); p[self.idx]=v
            if self.idx==0 and p.count()>1: p[p.count()-1]=v
            self.pi.setPolygon(p); self.cb(self.idx, v)
        return super().itemChange(c, v)
    def mousePressEvent(self, e):
        if e.button() == Qt.MouseButton.RightButton: self.cb(self.idx, None)
        else: super().mousePressEvent(e)

class ResultsTab(QWidget):
    def __init__(self):
        super().__init__(); layout=QVBoxLayout(self); self.tabs=QTabWidget(); layout.addWidget(self.tabs)
    def render(self, data):
        self.tabs.clear(); g={}
        for d in data: g.setdefault(d['load'],[]).append(d)
        
        for k, items in g.items():
            w=QWidget(); h=QHBoxLayout(w); ls=QListWidget(); ls.setFixedWidth(200)
            [ls.addItem(i['desc']) for i in items]
            
            scn=BaseGridScene(); view=RulerView(scn)
            txt=QTextEdit(); txt.setFont(QFont("Consolas",9)); txt.setFixedWidth(250)
            
            # View Selector
            cmb = QComboBox(); cmb.addItems(["Composite", "Sweep Steps"])
            chk_ed = QCheckBox("Edit Poly")
            if items:
                for l in items[0]['lidars']: cmb.addItem(f"Lidar: {l['name']}")
            
            mid=QWidget(); ml=QVBoxLayout(mid); ml.setContentsMargins(0,0,0,0)
            ml.addWidget(cmb); ml.addWidget(chk_ed); ml.addWidget(view)
            
            def on_sel(row=None, items=items, scn=scn, txt=txt, cmb=cmb, ls=ls, chk=chk_ed):
                if row is None: row=ls.currentRow()
                if row<0:return
                d=items[row]; scn.clear()
                mode = cmb.currentText()
                
                chk.setVisible(mode == "Composite")
                
                # Steps
                if 'steps' in d and mode == "Sweep Steps":
                    for s in d['steps']:
                        v=[QPointF(x*SCALE,-y*SCALE) for x,y in s.exterior.coords]
                        pi=QGraphicsPolygonItem(QPolygonF(v)); pi.setBrush(QBrush(QColor("orange"))); pi.setOpacity(0.05); pi.setZValue(-10); scn.addItem(pi)

                # Field
                if d['global'] and mode == "Composite":
                     # Draw Ignored Region (Gray)
                     if d.get('ignored_poly'):
                         ip = d['ignored_poly']
                         geoms = ip.geoms if hasattr(ip, 'geoms') else [ip]
                         for p in geoms:
                             v=[QPointF(x*SCALE,-y*SCALE) for x,y in p.exterior.coords]
                             pi=QGraphicsPolygonItem(QPolygonF(v)); pi.setBrush(QBrush(QColor("#505050"))); pi.setOpacity(0.5); pi.setPen(QPen(Qt.PenStyle.NoPen)); pi.setZValue(-15); scn.addItem(pi)

                     poly_list = list(d['global'].geoms) if d['global'].geom_type=='MultiPolygon' else [d['global']]
                     for i, p in enumerate(poly_list):
                         v=[QPointF(x*SCALE,-y*SCALE) for x,y in p.exterior.coords]
                         pi=QGraphicsPolygonItem(QPolygonF(v)); pi.setBrush(QBrush(QColor("#FFD700"))); pi.setOpacity(0.4); pi.setZValue(-10); scn.addItem(pi)
                         
                         if chk.isChecked():
                             def cb(v_idx, pos, p_idx=i):
                                 old_p = poly_list[p_idx]; coords = list(old_p.exterior.coords)
                                 
                                 def upd_lidars():
                                     for l in d['lidars']:
                                         if 'fov_poly' not in l: continue
                                         c = d['global'].intersection(l['fov_poly'])
                                         if d.get('load_poly'):
                                             c = c.difference(d['load_poly'])
                                             if l.get('shadow_poly'): c = c.difference(l['shadow_poly'])
                                         l['clip'] = c
                                         s_rot = np.radians(90 + l['mount'])
                                         loc = []
                                         if not c.is_empty:
                                             co, si = np.cos(-s_rot), np.sin(-s_rot)
                                             R = np.array([[co, -si], [si, co]])
                                             gs = c.geoms if c.geom_type == 'MultiPolygon' else [c]
                                             for g in gs:
                                                 loc.append((np.array(g.exterior.coords) - np.array(l['origin'])) @ R.T)
                                         l['local'] = loc

                                 if pos is None:
                                     if len(coords) > 4:
                                         del coords[v_idx]
                                         if v_idx == 0: coords[-1] = coords[0]
                                         poly_list[p_idx] = Polygon(coords)
                                         if len(poly_list) > 1: d['global'] = MultiPolygon(poly_list)
                                         else: d['global'] = poly_list[0]
                                         upd_lidars()
                                         on_sel()
                                     return
                                 wx, wy = pos.x()/SCALE, -pos.y()/SCALE
                                 if v_idx < len(coords):
                                     coords[v_idx] = (wx, wy)
                                     if v_idx == 0: coords[-1] = (wx, wy)
                                     if v_idx == len(coords)-1: coords[0] = (wx, wy)
                                     poly_list[p_idx] = Polygon(coords)
                                     if len(poly_list) > 1: d['global'] = MultiPolygon(poly_list)
                                     else: d['global'] = poly_list[0]
                                     
                                     upd_lidars()
                                     
                                     # Update Text
                                     tr=""; cols=['cyan','lime','magenta']
                                     for i_l,l in enumerate(d['lidars']):
                                         tr += f"\n--- {l['name']} ---\n"
                                         for kidx, loc in enumerate(l['local']):
                                             tr += f"poly {kidx}:\n"; 
                                             for px,py in loc: tr+=f"{px:.3f}, {py:.3f}\n"
                                     if 'dist_d' in d: tr = f"Safety Dist (D): {d['dist_d']:.3f} m\n" + tr
                                     txt.setText(tr)
                             
                             for k in range(len(v)-1):
                                 scn.addItem(EditHandle(v[k].x(), v[k].y(), pi, k, cb))
                     
                     # Safety Distance Arc & Dot
                     if 'traj' in d and len(d['traj']) > 0:
                         path = QPainterPath()
                         t_pts = d['traj']
                         path.moveTo(t_pts[0][0]*SCALE, -t_pts[0][1]*SCALE)
                         for px, py, _ in t_pts[1:]: path.lineTo(px*SCALE, -py*SCALE)
                         
                         pg_path = QGraphicsPathItem(path); pg_path.setPen(QPen(QColor("cyan"), 2, Qt.PenStyle.DashLine)); pg_path.setZValue(15); scn.addItem(pg_path)
                         
                         last = t_pts[-1]
                         dot = scn.addEllipse(last[0]*SCALE-4, -last[1]*SCALE-4, 8, 8, QPen(Qt.PenStyle.NoPen), QBrush(QColor("blue"))); dot.setZValue(16)
                     
                     if 'front_traj' in d and len(d['front_traj']) > 0:
                         path = QPainterPath()
                         t_pts = d['front_traj']
                         path.moveTo(t_pts[0][0]*SCALE, -t_pts[0][1]*SCALE)
                         for px, py in t_pts[1:]: path.lineTo(px*SCALE, -py*SCALE)
                         pg_path = QGraphicsPathItem(path); pg_path.setPen(QPen(QColor("lime"), 2, Qt.PenStyle.DashLine)); pg_path.setZValue(15); scn.addItem(pg_path)
                         last = t_pts[-1]
                         dot = scn.addEllipse(last[0]*SCALE-4, -last[1]*SCALE-4, 8, 8, QPen(Qt.PenStyle.NoPen), QBrush(QColor("lime"))); dot.setZValue(16)

                # Lidars
                tr=""; cols=['cyan','lime','magenta']
                for i,l in enumerate(d['lidars']):
                    # Marker (Always)
                    el=scn.addEllipse(l['origin'][0]*SCALE-5, -l['origin'][1]*SCALE-5, 10,10, QPen(QColor("white")), QBrush(QColor("white")))
                    el.setZValue(5)
                    
                    tr += f"\n--- {l['name']} ---\n"
                    for kidx, loc in enumerate(l['local']):
                        tr += f"poly {kidx}:\n"; 
                        for px,py in loc: tr+=f"{px:.3f}, {py:.3f}\n"

                    if mode == "Composite": continue
                    if mode == "Sweep Steps": continue
                    if mode.startswith("Lidar:") and mode != f"Lidar: {l['name']}": continue
                    
                    c=cols[i%3]; clip=l['clip']
                    if not clip.is_empty:
                        cs=clip.geoms if clip.geom_type=='MultiPolygon' else [clip]
                        for p in cs:
                             v=[QPointF(x*SCALE,-y*SCALE) for x,y in p.exterior.coords]
                             pi=QGraphicsPolygonItem(QPolygonF(v)); pi.setBrush(QBrush(QColor(c))); pi.setOpacity(0.5); pi.setZValue(-10); scn.addItem(pi)
                
                if 'dist_d' in d: tr = f"Safety Dist (D): {d['dist_d']:.3f} m\n" + tr
                txt.setText(tr)
                
                # Load
                if d.get('load_poly'):
                    c = "#2196F3" if d['load'] == 'Load1' else "#4CAF50"
                    lp = d['load_poly']
                    geoms = lp.geoms if hasattr(lp, 'geoms') else [lp]
                    for g in geoms:
                        if g.geom_type == 'Polygon':
                            v=[QPointF(x*SCALE,-y*SCALE) for x,y in g.exterior.coords]
                            pg=QGraphicsPolygonItem(QPolygonF(v)); pg.setBrush(QBrush(QColor(c))); pg.setOpacity(0.3); pg.setPen(QPen(QColor(c), 2)); pg.setZValue(20); scn.addItem(pg)
                        elif g.geom_type in ['LineString', 'LinearRing']:
                            path = QPainterPath(); pts=[QPointF(x*SCALE,-y*SCALE) for x,y in g.coords]
                            path.moveTo(pts[0]); [path.lineTo(p) for p in pts[1:]]
                            pg=QGraphicsPathItem(path); pg.setPen(QPen(QColor(c), 2)); pg.setZValue(20); scn.addItem(pg)

                # Base (Moved to end to overlap)
                if d['base']: 
                    bp = d['base']
                    geoms = bp.geoms if hasattr(bp, 'geoms') else [bp]
                    for g in geoms:
                        if g.geom_type == 'Polygon':
                            v=[QPointF(x*SCALE,-y*SCALE) for x,y in g.exterior.coords]
                            pg=QGraphicsPolygonItem(QPolygonF(v)); pg.setPen(QPen(QColor("white"),2,Qt.PenStyle.DotLine)); pg.setZValue(10); scn.addItem(pg)
                        elif g.geom_type in ['LineString', 'LinearRing']:
                            path = QPainterPath(); pts=[QPointF(x*SCALE,-y*SCALE) for x,y in g.coords]
                            path.moveTo(pts[0]); [path.lineTo(p) for p in pts[1:]]
                            pg=QGraphicsPathItem(path); pg.setPen(QPen(QColor("white"),2,Qt.PenStyle.DotLine)); pg.setZValue(10); scn.addItem(pg)
            
            ls.currentRowChanged.connect(on_sel)
            cmb.currentIndexChanged.connect(lambda: on_sel())
            chk_ed.toggled.connect(lambda: on_sel())
            if ls.count()>0: ls.setCurrentRow(0)
            h.addWidget(ls); h.addWidget(mid, 1); h.addWidget(txt); self.tabs.addTab(w, k)

# =====================================================================
# 7. TAB 4: HELP
# =====================================================================
class HelpTab(QWidget):
    def __init__(self):
        super().__init__()
        l = QVBoxLayout(self)
        t = QTextEdit()
        t.setReadOnly(True)
        t.setStyleSheet("background-color: #2b2b2b; color: #ddd; font-size: 14px; padding: 10px;")
        t.setHtml("""
        <h2 style='color:#4CAF50'>Safety Studio User Guide</h2>
        <p>This tool generates safety fields for mobile robots based on kinematic footprints and braking physics.</p>
        
        <h3 style='color:#2196F3'>1. Editor Tab</h3>
        <ul>
            <li><b>Footprint:</b> Import a DXF file defining the robot's base shape. Use 'Clr' to remove.</li>
            <li><b>Sensors:</b> Configure LiDAR placement (X,Y), Mounting Angle, FOV, and Range.</li>
            <li><b>Loads:</b> Load additional DXF shapes for L1/L2 configurations (e.g., pallets). Use 'Clr' to remove.</li>
        </ul>
        <p><i><b>DXF Assumption:</b> The DXF origin (0,0) is considered as the <b>base_link</b>. Design DXF files accordingly (both footprint and loads should be defined w.r.t base link).</i></p>
        
        <h3 style='color:#2196F3'>2. Gen Tab (Generation)</h3>
        <ul>
            <li><b>Physics Config:</b>
                <ul>
                    <li><b>Tr:</b> System reaction time (seconds).</li>
                    <li><b>a:</b> Deceleration limit (m/s²).</li>
                    <li><b>Ds:</b> Safety buffer distance (meters).</li>
                </ul>
            </li>
            <li><b>Plan Auto-Gen:</b>
                <ul>
                    <li><b>Type:</b> Enable/Disable generation for NoLoad, Load1, or Load2.</li>
                    <li><b>Count:</b> Number of cases to generate per load type.</li>
                    <li><b>Max V/W:</b> Maximum Linear (m/s) and Angular (deg/s) velocities for generation.</li>
                    <li><b>Shadow:</b> Enable shadow casting for Load1/Load2 (prevents fields from going through the load).</li>
                </ul>
            </li>
            <li><b>Motion Types:</b> Toggle Linear, In-Place, or Curve Turns.</li>
            <li><b>Patch Notches:</b> Enable post-processing to smooth out artifacts in rotation fields.</li>
            <li><b>Execute Batch:</b> Runs the simulation for all rows in the table.</li>
        </ul>
        
        <h3 style='color:#2196F3'>3. Results Tab</h3>
        <ul>
            <li><b>View:</b> Inspect the generated Global Safety Field (Yellow).</li>
            <li><b>Sensors:</b> See how the field is clipped per sensor (Cyan/Magenta/Lime).</li>
            <li><b>Check:</b> Verify the safety distance arc (Cyan Dash) matches the field boundary.</li>
            <li><b>Edit:</b> Enable 'Edit Poly' to manually adjust field vertices.</li>
        </ul>
        """)
        l.addWidget(t)

# =====================================================================
# 8. MAIN
# =====================================================================
class App(QMainWindow):
    def __init__(self):
        super().__init__(); self.resize(1400,900); self.setWindowTitle("Safety Studio V1.0")
        self.t=QTabWidget(); self.ed=EditorTab(self); self.gn=GenTab(self); self.rs=ResultsTab(); self.hl=HelpTab()
        self.t.addTab(self.ed,"Editor"); self.t.addTab(self.gn,"Gen"); self.t.addTab(self.rs,"Result"); self.t.addTab(self.hl,"Help")
        self.setCentralWidget(self.t)

    def run_sim(self):
        FootPrint=self.ed.polys['FootPrint']; sens=self.ed.get_s(); gn=self.gn
        if not FootPrint: QMessageBox.critical(self,"E","No FootPrint"); return
        
        try: P={
            'tr':float(gn.ptr.text()), 'ac':float(gn.pac.text()), 'ds':float(gn.pds.text()),
            'pad':float(gn.ppd.text()), 'smooth':float(gn.psm.text()), 
            'lat_scale':float(gn.lat_s.text()),
            'patch_notch': gn.c_notch.isChecked()
        }
        except: return
        
        ld_p={'NoLoad':None}
        if self.ed.polys['L1']: ld_p['Load1']=self.ed.polys['L1']
        if self.ed.polys['L2']: ld_p['Load2']=self.ed.polys['L2']
        
        res=[]; tbl=gn.tbl
        for r in range(tbl.rowCount()):
            try:
                ltype=tbl.item(r,1).text(); v=float(tbl.item(r,2).text()); w=float(tbl.item(r,3).text())
                lp = ld_p.get(ltype, None)
                
                if ltype == 'Load1': P['shadow'] = gn.sh_l1.isChecked()
                elif ltype == 'Load2': P['shadow'] = gn.sh_l2.isChecked()
                else: P['shadow'] = False
                
                w_in = np.radians(w) # Input is Deg
                gf, lid, trj, stp, val_d, ftrj, ign = SafetyMath.calc_case(FootPrint, lp, sens, v, w_in, P)
                if gf is None: continue
                
                bpoly=FootPrint
                if lp: bpoly=unary_union([bpoly, lp]).convex_hull
                
                res.append({'desc':f"{ltype} {v} {w}", 'load':ltype, 'global':gf, 'lidars':lid, 'base':bpoly, 'steps':stp, 'load_poly':lp, 'traj':trj, 'dist_d':val_d, 'front_traj':ftrj, 'ignored_poly':ign})
            except:pass
        self.rs.render(res); self.t.setCurrentIndex(2)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app=QApplication(sys.argv); app.setStyle("Fusion")
    pal=QPalette(); pal.setColor(QPalette.ColorRole.Window, QColor(50,50,50)); pal.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white); pal.setColor(QPalette.ColorRole.Base, QColor(35,35,35)); pal.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white); pal.setColor(QPalette.ColorRole.Button, QColor(50,50,50)); pal.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    app.setPalette(pal)
    w=App(); w.show(); sys.exit(app.exec())