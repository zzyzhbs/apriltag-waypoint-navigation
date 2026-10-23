import math
import os

import pytest

from robot_control.landmark_map import load_map, parse_map

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw2', 'landmarks.yaml')


def test_parse_map():
    m = parse_map({'tag_size': 0.166, 'landmarks': [
        {'id': 3, 'x': 1.0, 'y': 2.0, 'yaw_deg': 270}]})
    lm = m.get(3)
    assert m.tag_size == 0.166
    assert (lm.x, lm.y) == (1.0, 2.0)
    assert lm.yaw == pytest.approx(-math.pi / 2)    # wrapped
    assert m.get(9) is None


def test_parse_map_errors():
    with pytest.raises(ValueError):
        parse_map({'landmarks': []})
    with pytest.raises(ValueError):
        parse_map({'landmarks': [{'id': 0, 'x': 0, 'y': 0}]})
    with pytest.raises(ValueError):
        parse_map({'landmarks': [{'id': 0, 'x': 0, 'y': 0, 'yaw_deg': 0}] * 2})


def test_hw2_map_tags_face_their_leg():
    m = load_map(CONFIG)
    assert m.ids() == [0, 1, 2]
    legs = {0: ((0, 0), (1, 0)), 1: ((1, 0), (1, 2)), 2: ((1, 2), (0, 0))}
    for tag_id, ((x0, y0), (x1, y1)) in legs.items():
        lm = m.get(tag_id)
        heading = math.atan2(y1 - y0, x1 - x0)
        # 0.5 m past the leg's end, normal pointing back along the leg
        assert lm.x == pytest.approx(x1 + 0.5 * math.cos(heading), abs=1e-3)
        assert lm.y == pytest.approx(y1 + 0.5 * math.sin(heading), abs=1e-3)
        assert math.cos(lm.yaw - heading) == pytest.approx(-1.0, abs=1e-6)
