"""センサーの記録成功と、巡回教師としての採用条件を区別する。"""


def patrol_quality(summary, config):
    episodes = summary["patrol_episodes"]
    distance = sum(item["distance_m"] for item in episodes)
    moving_seconds = max(0, summary["simulated_seconds"] - 2 * len(episodes))
    minimum_distance = config.cruise_speed_m_s * 0.2 * moving_seconds
    clearances = [
        item["minimum_map_clearance_m"] for item in episodes if item["minimum_map_clearance_m"] is not None
    ]
    reasons = []
    if summary["falls"]:
        reasons.append("fall")
    if distance < minimum_distance:
        reasons.append("insufficient_patrol_motion")
    if not clearances or min(clearances) < config.robot_radius_m:
        reasons.append("insufficient_obstacle_clearance")
    return {
        "accepted": not reasons,
        "reasons": reasons,
        "distance_m": distance,
        "minimum_required_distance_m": minimum_distance,
        "minimum_map_clearance_m": min(clearances) if clearances else None,
        "required_obstacle_clearance_m": config.robot_radius_m,
    }
