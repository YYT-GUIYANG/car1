#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_trace.py 与 run_vision_on_video.py 不是同一文件：本脚本为「精确 HSV id + 几何比」新逻辑；后者为旧版族匹配流程。

核心
  1) 预处理 → Lab 分割 → 轮廓（带 lab_cid）
  2) 中心圆：圆度 + 面积/最小外接圆面积 + 面积/最小外接矩形面积 筛选后，取距画面中心最近；HSV 得 target_color_id
  3) 扇区：其余轮廓中 HSV id 与 target 完全相同；approx 顶点 4 或 5（epsilon 可调）；面积最大

滑条分 4 窗口：ColorSeg / CircleDet / SquareFilter / Preproc；滑条名与代码变量一致，窗内底板有公式图例（OpenCV 无真标签页，用多窗口代替）。

Canny/bridge、HSV 距离上限、proc_width_cap 仅命令行（避免单窗条数爆炸）。

交互：空格 暂停/继续；s 单步；k 将当前滑条与相关命令行参数写入文档；q/ESC 退出。视频文件播完自动从头循环；USB 摄像头为实时流（无循环）。

调试窗：Final Result / Mask View / Circle Candidates / Quad Candidates（与滑条窗独立）。
"""

from __future__ import annotations

import argparse
import json
import math
import shlex
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

REFERENCE_BGR: List[Tuple[int, Tuple[int, int, int]]] = [
    (0, (116, 5, 202)),
    (3, (167, 1, 98)),
    (4, (7, 237, 19)),
    (5, (23, 51, 215)),
    (6, (241, 132, 251)),
    (7, (17, 168, 214)),
    (8, (135, 199, 246)),
    (9, (221, 66, 76)),
    (10, (216, 199, 167)),
    (11, (85, 152, 55)),
    (12, (57, 244, 231)),
    (16, (23, 3, 23)),
]

COLOR_NAMES_EN: dict[int, str] = {
    0: "Deep purple",
    3: "Magenta / purple",
    4: "Green",
    5: "Blue",
    6: "Pink / light purple",
    7: "Cyan",
    8: "Light cyan",
    9: "Blue-red / target blue",
    10: "Beige",
    11: "Olive green",
    12: "Turquoise",
    16: "Near black",
}

WIN_COLOR = "ColorSeg"
WIN_CIRCLE = "CircleDet"
WIN_SQUARE = "SquareFilter"
WIN_PRE = "Preproc"
WIN_FINAL = "Final Result"
WIN_MASK = "Mask View"
WIN_CIRCLE_CAND = "Circle Candidates"
WIN_QUAD_CAND = "Quad Candidates"

# OpenCV 在滑条旁显示名称；短名更稳。含义见各窗底板图例（与代码变量名一致）。
TBC_LAB = "lab_thresh"
TBC_BLUR = "blur_ksize"
TBC_MORPH = "mask_morph"
TBC_MIN_AREA = "min_area"
TBR_CIRC = "center_circ_min"
TBR_FILL = "circle_area_ratio"
TBR_RECT = "rect_area_ratio_max"
TBR_DIST = "center_max_dist_r"
TBS_SMIN = "square_min_area"
TBS_SMAX = "square_max_area_r"
TBS_EPS = "square_approx_eps"
TBS_DIST = "square_max_dist_cr"
TBP_ROI = "roi_frac"
TBP_ZOOM = "center_zoom"
TBP_GAMMA = "gamma"
TBP_CLAHE = "clahe"


@dataclass
class Region:
    lab_cid: int = -1
    area: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    circularity: float = 0.0
    contour: np.ndarray = field(default_factory=lambda: np.zeros((0, 1, 2), dtype=np.int32))


def _noop(_: int) -> None:
    pass


def odd_1_15(x: int) -> int:
    x = int(max(1, min(15, x)))
    if x % 2 == 0:
        x = min(15, x + 1)
    return x


def contour_circularity(c: np.ndarray) -> float:
    a = float(cv2.contourArea(c))
    p = float(cv2.arcLength(c, True))
    if a < 1.0 or p < 1e-6:
        return 0.0
    return 4.0 * math.pi * a / (p * p)


def contour_area_to_circle_and_rect_ratios(c: np.ndarray) -> Tuple[float, float]:
    """返回 (轮廓面积/最小外接圆面积, 轮廓面积/最小外接矩形面积)。"""
    a = float(cv2.contourArea(c))
    if a < 1.0 or c.shape[0] < 3:
        return 0.0, 1.0
    (_, _), rad = cv2.minEnclosingCircle(c)
    ac = math.pi * max(rad, 1e-6) ** 2
    r_circ = a / ac
    rr = cv2.minAreaRect(c)
    w, h = rr[1]
    ar = max(float(w) * float(h), 1e-6)
    r_rect = a / ar
    return r_circ, r_rect


def approx_vertex_count_eps(c: np.ndarray, eps_frac: float) -> int:
    if c.shape[0] < 3:
        return 0
    per = cv2.arcLength(c, True)
    if per < 1e-6:
        return 0
    ef = max(0.005, min(0.08, eps_frac))
    ap = cv2.approxPolyDP(c, ef * per, True)
    return int(ap.shape[0])


def refine_mask(mask_u8: np.ndarray, ksize: int) -> np.ndarray:
    k = max(3, ksize | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    out = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)
    out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel)
    return out


def apply_gamma_bgr_inplace(bgr: np.ndarray, gamma: float) -> None:
    if bgr.size == 0 or abs(gamma - 1.0) < 0.02:
        return
    g = 1.0 / max(0.35, min(3.5, gamma))
    lut = np.clip(np.power(np.arange(256, dtype=np.float64) / 255.0, g) * 255.0, 0, 255).astype(np.uint8)
    cv2.LUT(bgr, lut, bgr)


def apply_clahe_lab_l_inplace(bgr: np.ndarray, clip_limit: float) -> None:
    if bgr.size == 0 or clip_limit <= 0.05:
        return
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=max(0.1, min(12.0, clip_limit)), tileGridSize=(8, 8))
    l2 = clahe.apply(l)
    lab2 = cv2.merge([l2, a, b])
    out = cv2.cvtColor(lab2, cv2.COLOR_LAB2BGR)
    np.copyto(bgr, out)


def center_crop_zoom_inplace(bgr: np.ndarray, zoom: float) -> None:
    if bgr.size == 0 or zoom <= 1.001:
        return
    vh, vw = bgr.shape[:2]
    cw = max(8, int(vw / zoom))
    ch = max(8, int(vh / zoom))
    x0 = (vw - cw) // 2
    y0 = (vh - ch) // 2
    crop = bgr[y0 : y0 + ch, x0 : x0 + cw].copy()
    cv2.resize(crop, (vw, vh), dst=bgr, interpolation=cv2.INTER_LINEAR)


def bgr_scalar_to_hsv(B: float, G: float, R: float) -> Tuple[float, float, float]:
    px = np.uint8([[[B, G, R]]])
    hsv = cv2.cvtColor(px, cv2.COLOR_BGR2HSV)
    t = hsv[0, 0]
    return float(t[0]), float(t[1]), float(t[2])


def hsv_dist_sq_color_table(h1: float, s1: float, v1: float, h2: float, s2: float, v2: float) -> float:
    dh = abs(h1 - h2)
    dh = min(dh, 180.0 - dh)
    ds = abs(s1 - s2)
    dv = abs(v1 - v2)
    hue_w, sat_w, val_w = 1.0, 0.3, 0.3
    a = hue_w * dh
    b = sat_w * ds
    c = val_w * dv
    return a * a + b * b + c * c


def classify_blob_mean_bgr_hsv_nearest(work_bgr: np.ndarray, r: Region, max_dist_sq: float) -> int:
    if r.contour.shape[0] < 3 or work_bgr.size == 0:
        return -1
    mask = np.zeros(work_bgr.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [r.contour], 0, 255, -1)
    mu = cv2.mean(work_bgr, mask)
    h0, s0, v0 = bgr_scalar_to_hsv(mu[0], mu[1], mu[2])
    best_id = -1
    best_d = 1e18
    for cid, ref in REFERENCE_BGR:
        rh, rs, rv = bgr_scalar_to_hsv(ref[0], ref[1], ref[2])
        d = hsv_dist_sq_color_table(h0, s0, v0, rh, rs, rv)
        if d < best_d:
            best_d = d
            best_id = cid
    if best_id < 0 or best_d > max_dist_sq:
        return -1
    return best_id


def build_labels_vectorized(lab: np.ndarray, ref_ids: np.ndarray, ref_lab: np.ndarray, thresh: float) -> np.ndarray:
    h, w = lab.shape[:2]
    pix = lab.reshape(-1, 3).astype(np.float32)
    d = np.linalg.norm(pix[:, None, :] - ref_lab[None, :, :], axis=2)
    best_d = np.min(d, axis=1)
    best_i = np.argmin(d, axis=1)
    out = np.where(best_d <= thresh, ref_ids[best_i], -1).astype(np.int32)
    return out.reshape(h, w)


def extract_regions(
    labels: np.ndarray,
    mask_k: int,
    min_area_collect: float,
    max_blobs_per_id: int,
    canny_edges: Optional[np.ndarray],
    canny_bridge: bool,
    bridge_dilate: int,
) -> List[Region]:
    regions: List[Region] = []
    for lab_cid, _ in REFERENCE_BGR:
        mask = (labels == lab_cid).astype(np.uint8) * 255
        mask = refine_mask(mask, mask_k)
        if canny_bridge and canny_edges is not None and canny_edges.size > 0:
            br = max(1, min(5, bridge_dilate))
            k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            md = cv2.dilate(mask, k5, iterations=br)
            add = cv2.bitwise_and(canny_edges, md)
            mask = cv2.bitwise_or(mask, add)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        scored: List[Tuple[float, np.ndarray]] = []
        for c in cnts:
            a = float(cv2.contourArea(c))
            if a < min_area_collect:
                continue
            scored.append((a, c))
        scored.sort(key=lambda x: -x[0])
        for a, c in scored[:max_blobs_per_id]:
            m = cv2.moments(c)
            if abs(m["m00"]) < 1e-6:
                continue
            reg = Region()
            reg.lab_cid = lab_cid
            reg.area = a
            reg.cx = m["m10"] / m["m00"]
            reg.cy = m["m01"] / m["m00"]
            reg.circularity = contour_circularity(c)
            reg.contour = c.reshape(-1, 1, 2).astype(np.int32)
            regions.append(reg)
    return regions


def build_merged_segmentation_mask(
    labels: np.ndarray,
    mask_k: int,
    canny_edges: Optional[np.ndarray],
    lab_mask_canny_bridge: bool,
    bridge_dilate: int,
) -> np.ndarray:
    """合并所有参考色 Lab 分割后的二值 mask（与 extract_regions 中每色 refine 一致，不按面积过滤）。"""
    h, w = labels.shape[:2]
    out = np.zeros((h, w), dtype=np.uint8)
    mk = odd_1_15(mask_k)
    for lab_cid, _ in REFERENCE_BGR:
        mask = (labels == lab_cid).astype(np.uint8) * 255
        mask = refine_mask(mask, mk)
        if lab_mask_canny_bridge and canny_edges is not None and canny_edges.size > 0:
            br = max(1, min(5, bridge_dilate))
            k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            md = cv2.dilate(mask, k5, iterations=br)
            add = cv2.bitwise_and(canny_edges, md)
            mask = cv2.bitwise_or(mask, add)
        out = cv2.bitwise_or(out, mask)
    return out


def collect_circle_geometry_candidates(
    regions: List[Region],
    opt_xy: Tuple[float, float],
    frame_area: float,
    min_side: float,
    center_circ_min: float,
    circle_area_ratio_min: float,
    rect_area_ratio_max: float,
    center_max_dist_ratio: float,
    min_area_lo: float,
    max_center_area_frac: float,
) -> List[Region]:
    """
    圆度 + 面积比 + 距中心距离上限；按距画面中心由近到远排序。
    """
    a_hi = max_center_area_frac * frame_area
    d_cap = center_max_dist_ratio * min_side
    scored: List[Tuple[float, Region]] = []
    for r in regions:
        if r.area < min_area_lo or r.area > a_hi:
            continue
        if r.circularity <= center_circ_min:
            continue
        rc, rr = contour_area_to_circle_and_rect_ratios(r.contour)
        if rc <= circle_area_ratio_min:
            continue
        if rr >= rect_area_ratio_max:
            continue
        dist = math.hypot(r.cx - opt_xy[0], r.cy - opt_xy[1])
        if dist > d_cap:
            continue
        scored.append((dist * dist, r))
    scored.sort(key=lambda x: x[0])
    return [r for _, r in scored]


def collect_quad_candidates_exact_hsv(
    regions: List[Region],
    center: Region,
    work_bgr: np.ndarray,
    target_color_id: int,
    square_min_area: float,
    square_max_area_ratio: float,
    square_approx_eps: float,
    square_max_dist_center_radius: float,
    sector_hsv_max_dist_sq: float,
    square_nv_min: int,
    square_nv_max: int,
    square_color_relax: float,
    square_max_area_center_mult: float,
    frame_area: float,
) -> List[Region]:
    """与扇区筛选相同条件，返回全部候选（不取面积最大）。"""
    def _blob_mean_hsv(reg: Region) -> Optional[Tuple[float, float, float]]:
        if reg.contour.shape[0] < 3 or work_bgr.size == 0:
            return None
        mask = np.zeros(work_bgr.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [reg.contour], 0, 255, -1)
        mu = cv2.mean(work_bgr, mask)
        return bgr_scalar_to_hsv(mu[0], mu[1], mu[2])

    def _target_hsv_dist_sq(hsv: Tuple[float, float, float], target_id: int) -> float:
        target_bgr: Optional[Tuple[int, int, int]] = None
        for cid, bgr in REFERENCE_BGR:
            if cid == target_id:
                target_bgr = bgr
                break
        if target_bgr is None:
            return 1e18
        rh, rs, rv = bgr_scalar_to_hsv(target_bgr[0], target_bgr[1], target_bgr[2])
        return hsv_dist_sq_color_table(hsv[0], hsv[1], hsv[2], rh, rs, rv)

    a_cap = square_max_area_ratio * frame_area
    center_radius = max(1.0, math.sqrt(max(center.area, 1.0) / math.pi))
    center_area = max(1.0, float(center.area))
    area_center_mult = max(1.0, float(square_max_area_center_mult))
    out: List[Region] = []
    nv_lo = max(3, int(square_nv_min))
    nv_hi = max(nv_lo, int(square_nv_max))
    # 颜色严格模式：宁可漏检也不误检。只有当 color_relax>1 时才允许“近似同色”兜底。
    color_relax = max(1.0, float(square_color_relax))
    for r in regions:
        if r is center:
            continue
        if r.area < square_min_area or r.area > a_cap:
            continue
        # 同色方块面积需接近中心圆：不允许超过中心面积的 area_center_mult 倍（默认 1.3）
        if r.area > area_center_mult * center_area:
            continue
        nv = approx_vertex_count_eps(r.contour, square_approx_eps)
        if nv < nv_lo or nv > nv_hi:
            continue
        hid = classify_blob_mean_bgr_hsv_nearest(work_bgr, r, sector_hsv_max_dist_sq)
        if hid != target_color_id:
            # 默认严格同色；需要时才显式放宽（color_relax>1）
            if color_relax <= 1.001:
                continue
            hsv = _blob_mean_hsv(r)
            if hsv is None:
                continue
            d_target = _target_hsv_dist_sq(hsv, target_color_id)
            if d_target > color_relax * sector_hsv_max_dist_sq:
                continue
        dist = math.hypot(r.cx - center.cx, r.cy - center.cy)
        side = math.sqrt(max(r.area, 1.0))
        dist_cap = max(max(0.8, square_max_dist_center_radius) * center_radius, 2.2 * side)
        if dist > dist_cap:
            continue
        out.append(r)
    return out


def draw_circle_candidates_view(
    work_bgr: np.ndarray,
    circle_cands: List[Region],
    chosen_center: Optional[Region],
) -> np.ndarray:
    vis = work_bgr.copy()
    for r in circle_cands:
        cv2.drawContours(vis, [r.contour], 0, (0, 255, 0), 1, lineType=cv2.LINE_AA)
    if chosen_center is not None and chosen_center.contour.shape[0] >= 3:
        (cx_f, cy_f), rad = cv2.minEnclosingCircle(chosen_center.contour)
        cx, cy, rr = int(cx_f), int(cy_f), int(max(3, round(rad)))
        cv2.circle(vis, (cx, cy), rr, (0, 255, 255), 3, lineType=cv2.LINE_AA)
    return vis


def draw_quad_candidates_view(
    work_bgr: np.ndarray,
    quad_cands: List[Region],
    chosen_sector: Optional[Region],
) -> np.ndarray:
    vis = work_bgr.copy()
    for r in quad_cands:
        cv2.drawContours(vis, [r.contour], 0, (255, 0, 0), 1, lineType=cv2.LINE_AA)
    if chosen_sector is not None and chosen_sector.contour.shape[0] >= 3:
        x, y, ww, hh = cv2.boundingRect(chosen_sector.contour)
        cv2.rectangle(vis, (x, y), (x + ww, y + hh), (0, 0, 0), 3, lineType=cv2.LINE_AA)
    return vis


def draw_overlay(
    dbg: np.ndarray,
    center: Optional[Region],
    sector: Optional[Region],
    target_color_id: int,
    status_lines: List[str],
) -> None:
    if center is not None and center.contour.shape[0] >= 3:
        (cx_f, cy_f), rad = cv2.minEnclosingCircle(center.contour)
        cx, cy, r = int(cx_f), int(cy_f), int(max(3, round(rad)))
        cv2.circle(dbg, (cx, cy), r, (0, 255, 255), 2, lineType=cv2.LINE_AA)

    if sector is not None and sector.contour.shape[0] >= 3:
        x, y, ww, hh = cv2.boundingRect(sector.contour)
        cv2.rectangle(dbg, (x, y), (x + ww, y + hh), (0, 0, 0), 2, lineType=cv2.LINE_AA)

    if target_color_id >= 0:
        name = COLOR_NAMES_EN.get(target_color_id, f"ID {target_color_id}")
        header = f"Center: {name} (ID {target_color_id})"
    elif center is not None:
        header = f"Center: HSV unmatched (lab id {center.lab_cid})"
    else:
        header = "Center: —"
    y0 = 22
    cv2.putText(dbg, header, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(dbg, header, (8, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    for i, t in enumerate(status_lines[:6]):
        y = y0 + 24 + i * 20
        cv2.putText(dbg, t[:110], (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(dbg, t[:110], (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 255, 200), 1, cv2.LINE_AA)


def run_offline_trace_frame(
    frame_bgr: np.ndarray,
    roi_frac: float,
    center_zoom: float,
    gamma: float,
    clahe_clip: float,
    proc_width_cap: int,
    lab_thresh: float,
    blur_ksize: int,
    mask_morph: int,
    lab_mask_canny_bridge: bool,
    canny_t1: int,
    canny_t2: int,
    canny_dilate_px: int,
    bridge_mask_dilate: int,
    min_area_seg: float,
    center_circ_min: float,
    circle_area_ratio_min: float,
    rect_area_ratio_max: float,
    center_max_dist_ratio: float,
    square_min_area: float,
    square_max_area_ratio: float,
    square_approx_eps: float,
    square_max_dist_center_radius: float,
    square_nv_min: int,
    square_nv_max: int,
    square_color_relax: float,
    square_max_area_center_mult: float,
    center_hsv_max_dist_sq: float,
    sector_hsv_max_dist_sq: float,
    max_center_area_frac: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, str, int]:
    h, w = frame_bgr.shape[:2]
    roi_w = int(w * roi_frac)
    roi_h = int(h * roi_frac)
    x0 = (w - roi_w) // 2
    y0 = (h - roi_h) // 2
    vis = frame_bgr[y0 : y0 + roi_h, x0 : x0 + roi_w].copy()

    center_crop_zoom_inplace(vis, center_zoom)
    apply_gamma_bgr_inplace(vis, gamma)
    apply_clahe_lab_l_inplace(vis, clahe_clip)

    work = vis
    if vis.shape[1] > proc_width_cap:
        sc = proc_width_cap / float(vis.shape[1])
        nw = int(vis.shape[1] * sc)
        nh = int(vis.shape[0] * sc)
        work = cv2.resize(vis, (nw, nh), interpolation=cv2.INTER_AREA)

    lab_src = work
    blur_use = odd_1_15(blur_ksize)
    if blur_use >= 3:
        lab_src = cv2.GaussianBlur(work, (blur_use, blur_use), 0)
    lab = cv2.cvtColor(lab_src, cv2.COLOR_BGR2LAB)

    ref_ids = np.array([cid for cid, _ in REFERENCE_BGR], dtype=np.int32)
    ref_lab = np.zeros((len(REFERENCE_BGR), 3), dtype=np.float32)
    for i, (_, bgr) in enumerate(REFERENCE_BGR):
        px = np.uint8([[[bgr[0], bgr[1], bgr[2]]]])
        lp = cv2.cvtColor(px, cv2.COLOR_BGR2LAB)
        ref_lab[i] = lp[0, 0].astype(np.float32)

    labels = build_labels_vectorized(lab, ref_ids, ref_lab, lab_thresh)

    mk = odd_1_15(mask_morph)
    canny_edges: Optional[np.ndarray] = None
    if lab_mask_canny_bridge and work.shape[0] > 8 and work.shape[1] > 8:
        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        t2 = max(canny_t2, canny_t1 + 1)
        ed = cv2.Canny(gray, float(canny_t1), float(t2))
        dpx = odd_1_15(canny_dilate_px)
        dk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dpx, dpx))
        canny_edges = cv2.dilate(ed, dk)

    merged_mask_u8 = build_merged_segmentation_mask(
        labels, mk, canny_edges, lab_mask_canny_bridge, bridge_mask_dilate
    )

    frame_area = float(work.shape[0] * work.shape[1])
    min_side = float(min(work.shape[0], work.shape[1]))
    min_collect = max(10.0, min_area_seg * 0.45)
    regions = extract_regions(
        labels,
        mk,
        min_collect,
        8,
        canny_edges,
        lab_mask_canny_bridge,
        bridge_mask_dilate,
    )

    opt_xy = (work.shape[1] * 0.5, work.shape[0] * 0.5)
    min_center_a = max(25.0, min_area_seg * 0.35)
    circle_cands = collect_circle_geometry_candidates(
        regions,
        opt_xy,
        frame_area,
        min_side,
        center_circ_min,
        circle_area_ratio_min,
        rect_area_ratio_max,
        center_max_dist_ratio,
        min_center_a,
        max_center_area_frac,
    )
    center_r = circle_cands[0] if circle_cands else None

    target_id = -1
    sector: Optional[Region] = None
    quad_cands: List[Region] = []
    status_lines: List[str] = []

    if center_r is None:
        status_lines.append("no center (geom/circ/dist)")
    else:
        target_id = classify_blob_mean_bgr_hsv_nearest(work, center_r, center_hsv_max_dist_sq)
        if target_id < 0:
            status_lines.append(f"HSV reject lab={center_r.lab_cid}")
        else:
            quad_cands = collect_quad_candidates_exact_hsv(
                regions,
                center_r,
                work,
                target_id,
                square_min_area,
                square_max_area_ratio,
                square_approx_eps,
                square_max_dist_center_radius,
                sector_hsv_max_dist_sq,
                square_nv_min,
                square_nv_max,
                square_color_relax,
                square_max_area_center_mult,
                frame_area,
            )
            sector = max(quad_cands, key=lambda r: r.area) if quad_cands else None
            status_lines.append(f"nreg={len(regions)} sector={'ok' if sector else 'none'}")

    dbg = work.copy()
    for r in regions:
        cv2.drawContours(dbg, [r.contour], 0, (90, 90, 90), 1)
    draw_overlay(dbg, center_r, sector, target_id, status_lines)

    vis_circle = draw_circle_candidates_view(work, circle_cands, center_r)
    vis_quad = draw_quad_candidates_view(work, quad_cands, sector)

    summary = f"id={target_id} n={len(regions)}"
    return dbg, merged_mask_u8, vis_circle, vis_quad, summary, target_id


def _control_panel_bg(win: str, legend_lines: List[str]) -> None:
    """深色底板 + 顶部图例（滑条在图像下方，自上而下与 createTrackbar 顺序一致）。"""
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    img = np.zeros((168, 580, 3), dtype=np.uint8)
    y = 14
    for line in legend_lines[:6]:
        cv2.putText(img, line[:70], (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
        y += 20
    cv2.imshow(win, img)


def setup_trackbars(args: argparse.Namespace) -> None:
    _control_panel_bg(
        WIN_COLOR,
        [
            "ColorSeg (top->bottom = slider order):",
            "1 lab_thresh: 30 + pos/10  (Lab L2 gate)",
            "2 blur_ksize: odd 1..15 from pos+1",
            "3 mask_morph: odd morph k from pos+1",
            "4 min_area: 50 + pos  (min blob for regions)",
        ],
    )
    lab_pos = int(round(max(0, min(700, (args.lab_thresh - 30.0) * 10.0))))
    cv2.createTrackbar(TBC_LAB, WIN_COLOR, lab_pos, 700, _noop)
    cv2.createTrackbar(TBC_BLUR, WIN_COLOR, max(0, min(14, args.blur - 1)), 14, _noop)
    cv2.createTrackbar(TBC_MORPH, WIN_COLOR, max(0, min(14, args.mask_morph - 1)), 14, _noop)
    cv2.createTrackbar(TBC_MIN_AREA, WIN_COLOR, max(0, min(750, int(args.min_area - 50))), 750, _noop)

    _control_panel_bg(
        WIN_CIRCLE,
        [
            "CircleDet (top->bottom):",
            "1 center_circ_min: 0.65 + pos/100",
            "2 circle_area_ratio: 0.70+pos/100 (A/minEnclCirc)",
            "3 rect_area_ratio_max: 0.75+pos/100 (A/minRect upper)",
            "4 center_max_dist_r: 0.08+pos/100 * min(H,W)",
        ],
    )
    cv2.createTrackbar(TBR_CIRC, WIN_CIRCLE, int((args.center_circ_min - 0.65) * 100), 27, _noop)
    cv2.createTrackbar(TBR_FILL, WIN_CIRCLE, int((args.circle_area_ratio_min - 0.70) * 100), 28, _noop)
    cv2.createTrackbar(TBR_RECT, WIN_CIRCLE, int((args.rect_area_ratio_max - 0.75) * 100), 24, _noop)
    cv2.createTrackbar(TBR_DIST, WIN_CIRCLE, int((args.center_max_dist_ratio - 0.08) * 100), 52, _noop)

    _control_panel_bg(
        WIN_SQUARE,
        [
            "SquareFilter (top->bottom):",
            "1 square_min_area: 50 + pos",
            "2 square_max_area_r: 0.25 + pos/100 of frame",
            "3 square_approx_eps: 0.01 + pos/1000 for approxPolyDP",
            "4 square_max_dist_cr: 0.8 + pos/10 (x center radius)",
        ],
    )
    cv2.createTrackbar(TBS_SMIN, WIN_SQUARE, max(0, min(750, int(args.square_min_area - 50))), 750, _noop)
    cv2.createTrackbar(TBS_SMAX, WIN_SQUARE, int((args.square_max_area_ratio - 0.25) * 100), 45, _noop)
    cv2.createTrackbar(TBS_EPS, WIN_SQUARE, int((args.square_approx_eps - 0.01) * 1000), 40, _noop)
    cv2.createTrackbar(TBS_DIST, WIN_SQUARE, int((args.square_max_dist_center_radius - 0.8) * 10), 52, _noop)

    _control_panel_bg(
        WIN_PRE,
        [
            "Preproc (top->bottom):",
            "1 roi_frac: (40+pos)/100  center crop",
            "2 center_zoom: 1 + pos/100  (crop zoom)",
            "3 gamma: 0.5 + pos/100",
            "4 clahe: pos/100 clip on L channel",
        ],
    )
    cv2.createTrackbar(TBP_ROI, WIN_PRE, int(args.roi_frac * 100 - 40), 55, _noop)
    cv2.createTrackbar(TBP_ZOOM, WIN_PRE, int((args.center_zoom - 1.0) * 100), 80, _noop)
    cv2.createTrackbar(TBP_GAMMA, WIN_PRE, int((args.gamma - 0.5) * 100), 150, _noop)
    cv2.createTrackbar(TBP_CLAHE, WIN_PRE, int(args.clahe * 100), 500, _noop)


def read_params_from_trackbars() -> dict:
    lab_t = 30.0 + cv2.getTrackbarPos(TBC_LAB, WIN_COLOR) / 10.0
    blur_v = odd_1_15(cv2.getTrackbarPos(TBC_BLUR, WIN_COLOR) + 1)
    morph_v = odd_1_15(cv2.getTrackbarPos(TBC_MORPH, WIN_COLOR) + 1)
    min_a = float(50 + cv2.getTrackbarPos(TBC_MIN_AREA, WIN_COLOR))

    circ_min = 0.65 + cv2.getTrackbarPos(TBR_CIRC, WIN_CIRCLE) / 100.0
    fill_circ = 0.70 + cv2.getTrackbarPos(TBR_FILL, WIN_CIRCLE) / 100.0
    rect_max = 0.75 + cv2.getTrackbarPos(TBR_RECT, WIN_CIRCLE) / 100.0
    dist_max = 0.08 + cv2.getTrackbarPos(TBR_DIST, WIN_CIRCLE) / 100.0

    sq_min = float(50 + cv2.getTrackbarPos(TBS_SMIN, WIN_SQUARE))
    sq_max_r = 0.25 + cv2.getTrackbarPos(TBS_SMAX, WIN_SQUARE) / 100.0
    sq_eps = 0.01 + cv2.getTrackbarPos(TBS_EPS, WIN_SQUARE) / 1000.0
    sq_dist_cr = 0.8 + cv2.getTrackbarPos(TBS_DIST, WIN_SQUARE) / 10.0

    roi_f = (40 + cv2.getTrackbarPos(TBP_ROI, WIN_PRE)) / 100.0
    zoom = 1.0 + cv2.getTrackbarPos(TBP_ZOOM, WIN_PRE) / 100.0
    gamma = 0.5 + cv2.getTrackbarPos(TBP_GAMMA, WIN_PRE) / 100.0
    clahe = cv2.getTrackbarPos(TBP_CLAHE, WIN_PRE) / 100.0

    return {
        "lab_thresh": lab_t,
        "blur_ksize": blur_v,
        "mask_morph": morph_v,
        "min_area": min_a,
        "center_circ_min": circ_min,
        "circle_area_ratio_min": fill_circ,
        "rect_area_ratio_max": rect_max,
        "center_max_dist_ratio": dist_max,
        "square_min_area": sq_min,
        "square_max_area_ratio": sq_max_r,
        "square_approx_eps": sq_eps,
        "square_max_dist_center_radius": sq_dist_cr,
        "roi_frac": roi_f,
        "center_zoom": zoom,
        "gamma": gamma,
        "clahe": clahe,
    }


def _fmt_cli_float(x: float) -> str:
    s = f"{x:.6g}"
    if "e" in s.lower():
        return repr(float(x))
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s if s else "0"


def build_reproduce_command_line(p: Dict[str, Any], args: argparse.Namespace) -> str:
    """生成与当前滑条等价的命令行（在项目目录下执行）。"""
    parts: List[str] = ["python3", "offline_trace.py"]
    if args.video:
        parts += ["--video", args.video]
    else:
        cam = 0 if args.camera is None else int(args.camera)
        parts += ["--camera", str(cam)]
    parts += ["--proc-width-cap", str(int(args.proc_width_cap))]
    parts += ["--lab-thresh", _fmt_cli_float(float(p["lab_thresh"]))]
    parts += ["--blur", str(int(p["blur_ksize"]))]
    parts += ["--mask-morph", str(int(p["mask_morph"]))]
    parts += ["--min-area", _fmt_cli_float(float(p["min_area"]))]
    parts += ["--center-circ-min", _fmt_cli_float(float(p["center_circ_min"]))]
    parts += ["--circle-area-ratio-min", _fmt_cli_float(float(p["circle_area_ratio_min"]))]
    parts += ["--rect-area-ratio-max", _fmt_cli_float(float(p["rect_area_ratio_max"]))]
    parts += ["--center-max-dist-ratio", _fmt_cli_float(float(p["center_max_dist_ratio"]))]
    parts += ["--square-min-area", _fmt_cli_float(float(p["square_min_area"]))]
    parts += ["--square-max-area-ratio", _fmt_cli_float(float(p["square_max_area_ratio"]))]
    parts += ["--square-approx-eps", _fmt_cli_float(float(p["square_approx_eps"]))]
    parts += ["--square-max-dist-center-radius", _fmt_cli_float(float(p["square_max_dist_center_radius"]))]
    parts += ["--square-nv-min", str(int(args.square_nv_min))]
    parts += ["--square-nv-max", str(int(args.square_nv_max))]
    parts += ["--square-color-relax", _fmt_cli_float(float(args.square_color_relax))]
    parts += ["--square-max-area-center-mult", _fmt_cli_float(float(args.square_max_area_center_mult))]
    parts += ["--roi-frac", _fmt_cli_float(float(p["roi_frac"]))]
    parts += ["--center-zoom", _fmt_cli_float(float(p["center_zoom"]))]
    parts += ["--gamma", _fmt_cli_float(float(p["gamma"]))]
    parts += ["--clahe", _fmt_cli_float(float(p["clahe"]))]
    parts += ["--center-hsv-max", _fmt_cli_float(float(args.center_hsv_max))]
    parts += ["--sector-hsv-max", _fmt_cli_float(float(args.sector_hsv_max))]
    parts += ["--canny-bridge", str(int(args.canny_bridge))]
    parts += ["--canny-t1", str(int(args.canny_t1))]
    parts += ["--canny-t2", str(int(args.canny_t2))]
    parts += ["--canny-dilate", str(int(args.canny_dilate))]
    parts += ["--bridge-mask-dilate", str(int(args.bridge_mask_dilate))]
    return " ".join(shlex.quote(str(x)) for x in parts)


def save_params_snapshot(
    save_dir: Path,
    track_params: Dict[str, Any],
    args: argparse.Namespace,
    source_label: str,
) -> Path:
    """
    将滑条解析后的参数 + 相关命令行项写入 UTF-8 文本；末尾附 JSON 便于程序读取。
    文件名：offline_trace_params_YYYYMMDD_HHMMSS.txt
    """
    save_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = save_dir / f"offline_trace_params_{ts}.txt"

    cli_only = {
        "proc_width_cap": int(args.proc_width_cap),
        "canny_bridge": int(args.canny_bridge),
        "canny_t1": int(args.canny_t1),
        "canny_t2": int(args.canny_t2),
        "canny_dilate": int(args.canny_dilate),
        "bridge_mask_dilate": int(args.bridge_mask_dilate),
        "center_hsv_max": float(args.center_hsv_max),
        "sector_hsv_max": float(args.sector_hsv_max),
        "square_nv_min": int(args.square_nv_min),
        "square_nv_max": int(args.square_nv_max),
        "square_color_relax": float(args.square_color_relax),
        "square_max_area_center_mult": float(args.square_max_area_center_mult),
        "max_center_area_frac": 0.22,
    }
    def _jsonable(v: Any) -> Any:
        if isinstance(v, (float, np.floating)):
            return float(v)
        if isinstance(v, (int, np.integer)):
            return int(v)
        return v

    payload = {
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "input_source": source_label,
        "from_trackbars": {k: _jsonable(v) for k, v in track_params.items()},
        "cli_only": cli_only,
    }

    cmd = build_reproduce_command_line(track_params, args)

    lines: List[str] = [
        "offline_trace.py 参数快照",
        f"保存时间: {payload['saved_at']}",
        f"输入源: {source_label}",
        "",
        "=== 滑条当前值（与界面一致，可直接对照变量名）===",
    ]
    for key in sorted(track_params.keys()):
        v = track_params[key]
        if isinstance(v, float):
            lines.append(f"{key} = {_fmt_cli_float(v)}")
        else:
            lines.append(f"{key} = {v}")

    lines += [
        "",
        "=== 仅命令行（无滑条）===",
    ]
    for key in sorted(cli_only.keys()):
        lines.append(f"{key} = {cli_only[key]}")

    lines += [
        "",
        "=== 等价启动命令（在项目「调试参数」目录执行）===",
        cmd,
        "",
        "=== JSON（整块可复制给脚本解析）===",
        json.dumps(payload, ensure_ascii=False, indent=2),
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def draw_param_strip(bgr: np.ndarray, p: dict, summary: str, paused: bool) -> None:
    lines = [
        f"Color lab={p['lab_thresh']:.1f} blur={p['blur_ksize']} morph={p['mask_morph']} minA={p['min_area']:.0f}",
        f"Circle circ>{p['center_circ_min']:.2f} fill>{p['circle_area_ratio_min']:.2f} rect<{p['rect_area_ratio_max']:.2f} d<{p['center_max_dist_ratio']:.2f}",
        f"Square A>={p['square_min_area']:.0f} maxR={p['square_max_area_ratio']:.2f} eps={p['square_approx_eps']:.3f} dcr<{p['square_max_dist_center_radius']:.1f}",
        f"Square nv=[{int(p['square_nv_min'])},{int(p['square_nv_max'])}] color_relax={p['square_color_relax']:.2f} area<={p['square_max_area_center_mult']:.2f}xC",
        f"Pre roi={p['roi_frac']:.2f} zoom={p['center_zoom']:.2f} g={p['gamma']:.2f} clahe={p['clahe']:.2f} [{'PAUSE' if paused else 'PLAY'}]",
        summary[:120],
    ]
    y0 = 100
    for i, t in enumerate(lines):
        y = y0 + i * 20
        cv2.putText(bgr, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(bgr, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)


def _read_frame(cap: cv2.VideoCapture, loop_on_eof: bool) -> Tuple[bool, np.ndarray]:
    """读取下一帧。loop_on_eof 为 True（视频文件）时读到末尾则回到开头；摄像头为 False。"""
    ok, frame = cap.read()
    if ok:
        return True, frame
    if not loop_on_eof:
        return False, frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    ok2, frame2 = cap.read()
    return ok2, frame2


def _resize_for_show(img: np.ndarray, sc: float) -> np.ndarray:
    if sc >= 0.99:
        return img
    ih, iw = img.shape[:2]
    nw, nh = int(iw * sc), int(ih * sc)
    interp = cv2.INTER_NEAREST if img.ndim == 2 else cv2.INTER_AREA
    return cv2.resize(img, (nw, nh), interpolation=interp)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="offline_trace：分窗滑条 + 圆面积比 + 精确 HSV 扇区；默认 USB 摄像头，也可用 --video 文件"
    )
    ap.add_argument(
        "--video",
        default="",
        help="视频文件路径；若指定则不用摄像头。与 --camera 不能同时用。",
    )
    ap.add_argument(
        "--camera",
        type=int,
        nargs="?",
        const=0,
        default=None,
        metavar="INDEX",
        help="USB 摄像头设备索引（默认 0）。单独写 --camera 等价于 --camera 0。与 --video 互斥。",
    )
    ap.add_argument("--out", default="")
    ap.add_argument(
        "--save-dir",
        default="",
        help="按 k 保存参数快照的目录；省略则保存到本脚本所在目录（调试参数文件夹）",
    )
    ap.add_argument("--proc-width-cap", type=int, default=960)
    ap.add_argument("--lab-thresh", type=float, default=71.5)
    ap.add_argument("--blur", type=int, default=15)
    ap.add_argument("--mask-morph", type=int, default=5)
    ap.add_argument("--min-area", type=float, default=50.0, help="ColorSeg：进入轮廓列表的最小面积下限")
    ap.add_argument("--center-circ-min", type=float, default=0.74)
    ap.add_argument("--circle-area-ratio-min", type=float, default=0.80)
    ap.add_argument("--rect-area-ratio-max", type=float, default=0.90)
    ap.add_argument("--center-max-dist-ratio", type=float, default=0.38)
    ap.add_argument("--square-min-area", type=float, default=220.0)
    ap.add_argument("--square-max-area-ratio", type=float, default=0.52)
    ap.add_argument("--square-approx-eps", type=float, default=0.024)
    ap.add_argument("--square-max-dist-center-radius", type=float, default=2.8)
    ap.add_argument("--square-nv-min", type=int, default=4)
    ap.add_argument("--square-nv-max", type=int, default=7)
    ap.add_argument(
        "--square-color-relax",
        type=float,
        default=1.0,
        help="方块颜色放宽倍数：1.0 为严格同色（宁可漏检不误检）；>1 才启用近似同色兜底",
    )
    ap.add_argument(
        "--square-max-area-center-mult",
        type=float,
        default=1.30,
        help="同色方块面积不得超过中心圆面积的倍数（默认 1.30）",
    )
    ap.add_argument("--roi-frac", type=float, default=0.70)
    ap.add_argument("--center-zoom", type=float, default=1.41)
    ap.add_argument("--gamma", type=float, default=0.76)
    ap.add_argument("--clahe", type=float, default=0.0)
    ap.add_argument("--center-hsv-max", type=float, default=5200.0)
    ap.add_argument("--sector-hsv-max", type=float, default=5200.0)
    ap.add_argument("--canny-bridge", type=int, default=0)
    ap.add_argument("--canny-t1", type=int, default=80)
    ap.add_argument("--canny-t2", type=int, default=160)
    ap.add_argument("--canny-dilate", type=int, default=5)
    ap.add_argument("--bridge-mask-dilate", type=int, default=2)
    args = ap.parse_args()

    if args.video and args.camera is not None:
        print("错误：不能同时使用 --video 与 --camera。", file=sys.stderr)
        sys.exit(2)

    if args.video:
        cap = cv2.VideoCapture(args.video)
        source_label = args.video
        loop_on_eof = True
    else:
        cam_idx = 0 if args.camera is None else int(args.camera)
        cap = cv2.VideoCapture(cam_idx)
        source_label = f"camera:{cam_idx}"
        loop_on_eof = False

    if not cap.isOpened():
        print("无法打开输入:", source_label, file=sys.stderr)
        sys.exit(1)

    save_dir = Path(args.save_dir).expanduser().resolve() if args.save_dir.strip() else Path(__file__).resolve().parent

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps is None or fps <= 1e-3:
        fps = 30.0
    for wn in (WIN_FINAL, WIN_MASK, WIN_CIRCLE_CAND, WIN_QUAD_CAND):
        cv2.namedWindow(wn, cv2.WINDOW_NORMAL)
    setup_trackbars(args)

    paused = False
    last_frame: Optional[np.ndarray] = None
    writer_out: Optional[cv2.VideoWriter] = None
    layout_done = False

    while True:
        if not paused:
            ok, frame = _read_frame(cap, loop_on_eof)
            if not ok:
                break
            last_frame = frame.copy()
        else:
            if last_frame is None:
                ok, frame = _read_frame(cap, loop_on_eof)
                if not ok:
                    break
                last_frame = frame.copy()
            else:
                frame = last_frame.copy()

        p = read_params_from_trackbars()
        p["square_nv_min"] = int(args.square_nv_min)
        p["square_nv_max"] = int(args.square_nv_max)
        p["square_color_relax"] = float(args.square_color_relax)
        p["square_max_area_center_mult"] = float(args.square_max_area_center_mult)
        dbg, mask_u8, vis_circle, vis_quad, summary, _ = run_offline_trace_frame(
            frame,
            roi_frac=p["roi_frac"],
            center_zoom=p["center_zoom"],
            gamma=p["gamma"],
            clahe_clip=p["clahe"],
            proc_width_cap=args.proc_width_cap,
            lab_thresh=p["lab_thresh"],
            blur_ksize=p["blur_ksize"],
            mask_morph=p["mask_morph"],
            lab_mask_canny_bridge=bool(args.canny_bridge),
            canny_t1=args.canny_t1,
            canny_t2=args.canny_t2,
            canny_dilate_px=args.canny_dilate,
            bridge_mask_dilate=args.bridge_mask_dilate,
            min_area_seg=p["min_area"],
            center_circ_min=p["center_circ_min"],
            circle_area_ratio_min=p["circle_area_ratio_min"],
            rect_area_ratio_max=p["rect_area_ratio_max"],
            center_max_dist_ratio=p["center_max_dist_ratio"],
            square_min_area=p["square_min_area"],
            square_max_area_ratio=p["square_max_area_ratio"],
            square_approx_eps=p["square_approx_eps"],
            square_max_dist_center_radius=p["square_max_dist_center_radius"],
            square_nv_min=p["square_nv_min"],
            square_nv_max=p["square_nv_max"],
            square_color_relax=p["square_color_relax"],
            square_max_area_center_mult=p["square_max_area_center_mult"],
            center_hsv_max_dist_sq=args.center_hsv_max,
            sector_hsv_max_dist_sq=args.sector_hsv_max,
            max_center_area_frac=0.22,
        )

        show = dbg.copy()
        draw_param_strip(show, p, summary, paused)

        dh, dw = show.shape[:2]
        sc = min(1100.0 / max(dw, 1), 820.0 / max(dh, 1), 2.0)
        disp = _resize_for_show(show, sc)
        disp_m = _resize_for_show(mask_u8, sc)
        disp_c = _resize_for_show(vis_circle, sc)
        disp_q = _resize_for_show(vis_quad, sc)

        cv2.imshow(WIN_FINAL, disp)
        cv2.imshow(WIN_MASK, disp_m)
        cv2.imshow(WIN_CIRCLE_CAND, disp_c)
        cv2.imshow(WIN_QUAD_CAND, disp_q)

        if not layout_done:
            gap = 24
            h2, w2 = disp.shape[:2]
            bx, by = 40, 40
            cv2.moveWindow(WIN_FINAL, bx, by)
            cv2.moveWindow(WIN_MASK, bx + w2 + gap, by)
            cv2.moveWindow(WIN_CIRCLE_CAND, bx, by + h2 + 48)
            cv2.moveWindow(WIN_QUAD_CAND, bx + w2 + gap, by + h2 + 48)
            layout_done = True

        if args.out:
            if writer_out is None:
                h2, w2 = disp.shape[:2]
                writer_out = cv2.VideoWriter(
                    args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w2, h2)
                )
            writer_out.write(disp)

        delay = 1 if not paused else 50
        k = cv2.waitKey(delay) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord(" "):
            paused = not paused
        if k == ord("s"):
            ok_s, nf = _read_frame(cap, loop_on_eof)
            if ok_s:
                last_frame = nf.copy()
                paused = True
                continue

        if k in (ord("k"), ord("K")):
            p_save = read_params_from_trackbars()
            try:
                snap = save_params_snapshot(save_dir, p_save, args, source_label)
                print("参数已保存:", snap, flush=True)
            except OSError as exc:
                print("保存参数失败:", exc, file=sys.stderr, flush=True)

    cap.release()
    if writer_out is not None:
        writer_out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
