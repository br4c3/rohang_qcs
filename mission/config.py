import json
import math

MAV_CMD_NAV_LAND = 21


def number(value, field, item_number, *, allow_null=False):
    if value is None and allow_null:
        return math.nan
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{item_number}번 항목의 {field} 값이 숫자가 아닙니다")
    return float(value)


def load_qgc_plan(path):
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"미션 파일을 찾을 수 없습니다: {path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"올바른 QGC JSON 파일이 아닙니다: {path}") from error

    if not isinstance(raw, dict) or raw.get("fileType") != "Plan":
        raise ValueError(f"QGroundControl Plan 파일이 아닙니다: {path}")

    mission = raw.get("mission")
    items = mission.get("items") if isinstance(mission, dict) else None
    if not isinstance(items, list) or not items:
        raise ValueError(f"미션 항목이 없는 Plan 파일입니다: {path}")

    waypoints = []
    coordinates = []
    for index, item in enumerate(items, start=1):
        if not isinstance(item, dict) or item.get("type") != "SimpleItem":
            raise ValueError(
                f"{path}: {index}번 항목은 지원하지 않는 ComplexItem입니다. "
                "QGC에서 SimpleItem(일반 미션 항목)으로 작성해 주세요."
            )

        params = item.get("params")
        if not isinstance(params, list) or len(params) != 7:
            raise ValueError(f"{path}: {index}번 항목의 params 형식이 잘못되었습니다")

        latitude = number(params[4], "위도", index)
        longitude = number(params[5], "경도", index)
        altitude = number(params[6], "고도", index)
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError(
                f"{path}: {index}번 항목의 위도/경도 범위가 잘못되었습니다"
            )

        coordinates.append((latitude, longitude))
        waypoints.append(
            {
                "frame": int(item.get("frame", 3)),
                "command": int(item["command"]),
                "is_current": index == 1,
                "autocontinue": bool(item.get("autoContinue", True)),
                "param1": number(params[0], "param1", index, allow_null=True),
                "param2": number(params[1], "param2", index, allow_null=True),
                "param3": number(params[2], "param3", index, allow_null=True),
                "param4": number(params[3], "param4", index, allow_null=True),
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
            }
        )

    summary = (path.resolve(), len(waypoints), coordinates[0], coordinates[-1])
    return waypoints, summary


def validate_hover_plan(waypoints):
    final_waypoint = waypoints[-1]
    if final_waypoint["command"] == MAV_CMD_NAV_LAND:
        raise ValueError("마지막 미션 항목은 Land일 수 없습니다. Waypoint로 변경하세요")
    if final_waypoint["altitude"] <= 0:
        raise ValueError("마지막 waypoint의 호버링 고도는 0보다 커야 합니다")
