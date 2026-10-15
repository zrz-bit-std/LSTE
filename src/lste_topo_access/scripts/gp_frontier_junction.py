"""One focused responsibility extracted from the legacy GP frontier node."""

import json
import math
import os

import numpy as np
import rospy

from gp_frontier_common import angle_diff, wrap_angle

class GpFrontierJunctionMixin:
    def _angle_mean(self, angles, weights):
        c = np.sum(np.cos(angles) * weights)
        s = np.sum(np.sin(angles) * weights)
        return wrap_angle(np.arctan2(s, c))

    def cluster_frontiers(self):
        """把本轮 frontier 按相近方向聚成峰，供路口判定使用。

        单个 frontier 可能只是噪声。多个分离的稳定峰持续出现，才可能表示
        左右岔路等真实分支；最终的时间稳定性判断在
        ``create_interest_from_frontiers``。返回值为 ``center`` 与 ``weight``
        组成的角度簇列表。
        """
        if self.gp_nav_frntr_cntrs is None:
            return []
        try:
            thetas = np.array(self.gp_nav_frntr_cntrs).reshape(-1, 2)[:, 0]
        except Exception:
            return []
        if thetas.size == 0:
            return []
        areas = getattr(self, "gp_nav_frntr_areas", None)
        if areas is None:
            weights = np.ones_like(thetas)
        else:
            try:
                weights = np.array(areas).reshape(-1)
            except Exception:
                weights = np.ones_like(thetas)
            if weights.size == 0:
                weights = np.ones_like(thetas)
            elif weights.size != thetas.size:
                # fallback to uniform if dimension mismatch
                weights = np.ones_like(thetas)
        # 按角度排序
        idx = np.argsort(thetas)
        thetas = thetas[idx]
        weights = weights[idx]
        clusters = []
        current_angles = [thetas[0]]
        current_weights = [weights[0]]
        for ang, w in zip(thetas[1:], weights[1:]):
            center_now = self._angle_mean(np.array(current_angles), np.array(current_weights))
            if angle_diff(ang, center_now) <= self.cluster_eps_rad:
                current_angles.append(ang)
                current_weights.append(w)
            else:
                clusters.append({"angles": current_angles, "weights": current_weights})
                current_angles = [ang]
                current_weights = [w]
        clusters.append({"angles": current_angles, "weights": current_weights})
        # wrap 合并首尾
        if len(clusters) > 1:
            first_c = self._angle_mean(np.array(clusters[0]["angles"]), np.array(clusters[0]["weights"]))
            last_c = self._angle_mean(np.array(clusters[-1]["angles"]), np.array(clusters[-1]["weights"]))
            if angle_diff(first_c, last_c) <= self.cluster_eps_rad:
                merged_angles = clusters[0]["angles"] + clusters[-1]["angles"]
                merged_weights = clusters[0]["weights"] + clusters[-1]["weights"]
                clusters = [{"angles": merged_angles, "weights": merged_weights}] + clusters[1:-1]
        result = []
        for cl in clusters:
            angs = np.array(cl["angles"])
            wts = np.array(cl["weights"])
            result.append({"center": self._angle_mean(angs, wts), "weight": float(wts.sum())})
        return result

    def update_cluster_track(self, clusters):
        """维护 cluster 跟踪，返回当前稳定的 cluster 列表。"""
        now = rospy.Time.now().to_sec()
        # 匹配已有 track
        for tr in self.cluster_track:
            tr["matched"] = False
        for cl in clusters:
            best = None
            best_diff = None
            for tr in self.cluster_track:
                diff = angle_diff(cl["center"], tr["center"])
                if diff <= self.cluster_eps_rad and (best_diff is None or diff < best_diff):
                    best = tr
                    best_diff = diff
            if best is not None:
                best["center"] = cl["center"]
                best["last_seen"] = now
                best["weight"] = cl["weight"]
                best["matched"] = True
            else:
                self.cluster_track.append({
                    "center": cl["center"],
                    "first_seen": now,
                    "last_seen": now,
                    "weight": cl["weight"],
                    "matched": True,
                })
        # 清理长时间未出现的
        self.cluster_track = [
            tr for tr in self.cluster_track
            if (now - tr.get("last_seen", now)) <= self.cluster_lost_timeout
        ]
        stable = []
        for tr in self.cluster_track:
            if (now - tr.get("first_seen", now)) >= self.cluster_stable_duration:
                stable.append({"center": tr["center"], "weight": tr.get("weight", 1.0)})
        return stable

    def create_interest_from_frontiers(self, chosen_theta_rel: float):
        """根据“多峰 -> 单峰”过程更新路口分支状态。

        进入路口时先记录所有稳定峰；机器人继续前进一段距离或超时后，再用
        实际运动方向确认哪一支已走过（CHOSEN），其余方向保留为 PENDING，
        以后可作为回退后的待探索分支。
        """
        now = rospy.Time.now().to_sec()
        clusters = self.cluster_frontiers()
        # 维护 cluster 跟踪，用于判定“多峰是否持续存在”
        self.update_cluster_track(clusters)
        # 只统计当前帧 matched 且已达到最小存在时长的峰，避免用到历史残留
        stable_present = []
        for tr in self.cluster_track:
            if not tr.get("matched", False):
                continue
            if (now - tr.get("first_seen", now)) < getattr(self, "junction_peak_min_age", 0.0):
                continue
            stable_present.append({"center": tr.get("center"), "weight": tr.get("weight", 1.0)})
        min_w = float(getattr(self, "junction_peak_min_weight", 0.0))
        if min_w > 0.0:
            stable_present = [cl for cl in stable_present if float(cl.get("weight", 0.0)) >= min_w]
        stable_present.sort(key=lambda x: float(x.get("weight", 0.0)), reverse=True)
        stable_count = len(stable_present)

        # 多峰持续判定：双峰满足分离/权重比，且持续足够长才进入会话
        multi_peak_now = False
        if stable_count >= 2:
            c0, c1 = stable_present[0], stable_present[1]
            sep = angle_diff(float(c0.get("center", 0.0)), float(c1.get("center", 0.0)))
            if sep >= float(getattr(self, "junction_peak_min_sep_rad", self.cluster_eps_rad)):
                w0 = float(c0.get("weight", 0.0))
                w1 = float(c1.get("weight", 0.0))
                ratio = float(getattr(self, "junction_second_weight_ratio", 0.0))
                if ratio <= 0.0 or (w0 > 1e-6 and (w1 / w0) >= ratio):
                    multi_peak_now = True
        if multi_peak_now:
            if self.multi_peak_since is None:
                self.multi_peak_since = now
        else:
            self.multi_peak_since = None
        enter_junction_session = (
            multi_peak_now
            and (self.multi_peak_since is not None)
            and ((now - self.multi_peak_since) >= float(getattr(self, "junction_enter_confirm_sec", 0.0)))
        )

        try:
            thetas_rel_raw = np.array(self.gp_nav_frntr_cntrs).reshape(-1, 2)[:, 0]
        except Exception:
            thetas_rel_raw = np.array([])

        stable_world = [
            {"heading": wrap_angle(self.pose.theta + float(cl.get("center", 0.0))), "weight": float(cl.get("weight", 0.0))}
            for cl in stable_present
        ]

        # 进入路口会话：满足双峰条件且通过冷却窗口
        if enter_junction_session:
            node_id = self.ensure_anchor(force=False)
            if node_id is not None:
                already = any(item.get("node_id") == node_id for item in self.pending_junctions)
                in_cooldown = False
                if self.last_finalized_anchor is not None and (node_id - self.last_finalized_anchor) <= self.junction_post_finalize_cooldown:
                    in_cooldown = True
                if self.last_junction_anchor is not None and (node_id - self.last_junction_anchor) < self.junction_cooldown_anchors:
                    in_cooldown = True
                if (not already) and (not in_cooldown):
                    anchor = self._get_node(node_id)
                    anchor_xy = (anchor["x"], anchor["y"]) if anchor else (
                        getattr(self.pose, "x", 0.0),
                        getattr(self.pose, "y", 0.0)
                    )
                    pj = {
                        "node_id": node_id,
                        "start_anchor": node_id,
                        "anchor_xy": anchor_xy,
                        "created": now,
                        "observations": [],
                        "actual_headings": [],
                        "peak_count_max": stable_count,
                        "was_multi": True,
                    }
                    self.pending_junctions.append(pj)
                    self.last_junction_anchor = node_id
                    self._maybe_init_junction_log(pj)

        # 会话中：持续记录峰与位移方向，双峰降为单峰时收尾
        new_pending = []
        for pj in self.pending_junctions:
            node_id = pj.get("node_id")
            anchor_xy = pj.get("anchor_xy", (0.0, 0.0))
            dist_from_anchor = None
            dx = dy = 0.0
            if self.pose is not None:
                dx = self.pose.x - anchor_xy[0]
                dy = self.pose.y - anchor_xy[1]
                dist_from_anchor = np.hypot(dx, dy)
                if dist_from_anchor > 0.05:  # 抖动过滤
                    try:
                        heading_actual = math.atan2(dy, dx)
                        pj.setdefault("actual_headings", []).append(heading_actual)
                    except Exception:
                        pass

            if stable_world:
                pj.setdefault("observations", []).extend(
                    [(float(sw["heading"]), float(sw["weight"])) for sw in stable_world]
                )
            pj["peak_count_max"] = max(pj.get("peak_count_max", 0), stable_count)
            # 单峰稳定计时：稳定 1 个峰持续到阈值才允许退出
            if stable_count == 1:
                if pj.get("single_peak_since") is None:
                    pj["single_peak_since"] = now
            else:
                pj["single_peak_since"] = None

            if thetas_rel_raw is not None and thetas_rel_raw.size > 0:
                self._append_junction_log(pj, now, thetas_rel_raw, self.pose)

            ended = (
                pj.get("was_multi", False)
                and stable_count == 1
                and pj.get("single_peak_since") is not None
                and (now - pj["single_peak_since"]) >= self.junction_exit_confirm_sec
            )
            if ended:
                self._finalize_junction_session(pj)
                self._finalize_junction_log(pj, status="finalized")
                continue  # 会话已结束，不再保留

            new_pending.append(pj)

        self.pending_junctions = new_pending

    def _prune_neighbor_branches(self, target_anchor: int, window: int = 3):
        """在 anchor 窗口内只保留 id 较大的分叉，清除窗口内更小 id 的分支。"""
        if target_anchor is None:
            return
        low = target_anchor - window
        for node in self.anchor_nodes:
            nid = node.get("id")
            if nid is None or nid == target_anchor:
                continue
            if nid < low or nid > target_anchor:
                continue
            if node.get("branches"):
                node["branches"] = []

    def _cluster_angles_with_weights(self, angles, weights):
        """把角度列表按 cluster_eps_rad 聚成簇，返回 [{'center','weight'}]."""
        if not angles:
            return []
        pairs = sorted(zip(angles, weights), key=lambda t: t[0])
        clusters = []

        def flush(cur_angles, cur_weights):
            if not cur_angles:
                return None
            center = self._angle_mean(np.array(cur_angles), np.array(cur_weights))
            weight = float(np.sum(cur_weights))
            return {"center": center, "weight": weight}

        cur_angles = []
        cur_weights = []
        for ang, w in pairs:
            if not cur_angles:
                cur_angles = [ang]
                cur_weights = [w]
                continue
            if angle_diff(ang, cur_angles[-1]) <= self.cluster_eps_rad:
                cur_angles.append(ang)
                cur_weights.append(w)
            else:
                cl = flush(cur_angles, cur_weights)
                if cl:
                    clusters.append(cl)
                cur_angles = [ang]
                cur_weights = [w]
        cl = flush(cur_angles, cur_weights)
        if cl:
            clusters.append(cl)

        # 合并首尾（环状角度），避免 0/2pi 断裂
        if len(clusters) > 1:
            first = clusters[0]
            last = clusters[-1]
            if angle_diff(first["center"], last["center"]) <= self.cluster_eps_rad:
                merged_center = self._angle_mean(
                    np.array([first["center"], last["center"]]),
                    np.array([first["weight"], last["weight"]])
                )
                merged_weight = first["weight"] + last["weight"]
                clusters = clusters[1:-1]
                clusters.append({"center": merged_center, "weight": merged_weight})

        clusters.sort(key=lambda x: x.get("weight", 0.0), reverse=True)
        return clusters

    def _finalize_junction_session(self, session: dict):
        """会话结束：按退出方向定 chosen，其余为 pending（限制数量）。"""
        start_anchor = session.get("start_anchor", session.get("node_id"))
        observations = session.get("observations", [])
        if not observations:
            rospy.logwarn_throttle(5.0, "junction finalize: no observations for node %s", start_anchor)
            return
        angles, weights = zip(*observations)
        clusters = self._cluster_angles_with_weights(list(angles), list(weights))
        if not clusters:
            rospy.logwarn_throttle(5.0, "junction finalize: no clusters for node %s", start_anchor)
            return

        # 用会话后半段位移方向作为退出方向
        heading_exit = None
        actual_headings = session.get("actual_headings", [])
        if actual_headings:
            tail = actual_headings[-4:] if len(actual_headings) >= 4 else actual_headings
            try:
                heading_exit = self._angle_mean(np.array(tail), np.ones(len(tail)))
            except Exception:
                heading_exit = None
        if heading_exit is None:
            heading_exit = clusters[0]["center"]

        chosen_idx = min(range(len(clusters)), key=lambda i: angle_diff(clusters[i]["center"], heading_exit))
        chosen = clusters.pop(chosen_idx)

        pending_limit = 2 if session.get("peak_count_max", 2) >= 3 else 1
        pending_clusters = sorted(clusters, key=lambda c: c.get("weight", 0.0), reverse=True)[:pending_limit]
        if not pending_clusters and clusters:
            pending_clusters = [clusters[0]]

        if not pending_clusters:
            rospy.logwarn_throttle(5.0, "junction finalize: no pending branches for node %s", start_anchor)

        # 目标 anchor：起始 anchor 与当前 anchor 的中位 id，找不到则退回 start
        end_anchor = self.anchor_last_id if self.anchor_last_id is not None else start_anchor
        target_anchor = start_anchor
        if end_anchor is not None and start_anchor is not None:
            try:
                if end_anchor < start_anchor:
                    end_anchor = start_anchor
                target_anchor = int((start_anchor + end_anchor) // 2)
            except Exception:
                target_anchor = start_anchor
        if self._get_node(target_anchor) is None:
            if end_anchor is not None and self._get_node(end_anchor) is not None:
                target_anchor = end_anchor
            else:
                target_anchor = start_anchor

        # 清除邻近 anchor（id 差 <=3）里更小 id 的分支，保留当前 anchor
        self._prune_neighbor_branches(target_anchor, window=3)

        for pc in pending_clusters:
            self._add_branch_if_new(target_anchor, pc.get("center"), pc.get("weight", 1.0))

        node = self._get_node(target_anchor)
        if node is not None:
            node["chosen_heading"] = chosen.get("center")
        # 记录用于冷却
        self.last_finalized_anchor = target_anchor

