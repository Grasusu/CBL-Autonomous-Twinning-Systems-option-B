import math

from tb3_pesticide_dt.pesticide_logic import (
    build_zones,
    classify_plant_health,
    normalize_angle,
)


def test_classify_plant_health_threshold():
    assert classify_plant_health(0.49, 0.50) == "OK"
    assert classify_plant_health(0.50, 0.50) == "TREATMENT_NEEDED"


def test_normalize_angle_range():
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, rel_tol=0.0, abs_tol=1e-9)
    assert -math.pi <= normalize_angle(-4.0) <= math.pi


def test_build_zones_rejects_mismatched_arrays():
    try:
        build_zones(["a"], ["A"], [0.0, 1.0], [0.0], [0.0], [0.1], ["OK"])
    except ValueError as exc:
        assert "equal length" in str(exc)
    else:
        raise AssertionError("Expected ValueError")
