from pathlib import Path

import pytest

from world_model_dataset.io import read_json
from world_model_dataset.showcase import DEFAULT_CONFIG, select_slices


def test_m5a_config_covers_each_core_event_once():
    config = read_json(DEFAULT_CONFIG)
    slices = config["slices"]
    assert len(slices) == 10
    assert len({row["id"] for row in slices}) == 10
    assert {row["event_id"] for row in slices} == {
        "R01", "R02", "R03", "R04", "R05", "V01", "V02", "V03", "V04", "V05"
    }
    assert set(config["event_framing"]) == {row["event_id"] for row in slices}
    assert all(not Path(row["episode"]).is_absolute() for row in slices)
    assert all(row["end_s"] > row["start_s"] >= 0 for row in slices)


def test_m5a_priority_and_explicit_selection():
    config = read_json(DEFAULT_CONFIG)
    priority = select_slices(config, None, "real_asset_delivery")
    assert [row["event_id"] for row in priority] == ["R02", "R03", "R04", "R05"]
    explicit = select_slices(config, ["V05_rigid_soft_impact", "R01_sphere_ramp"], None)
    assert [row["event_id"] for row in explicit] == ["R01", "V05"]
    with pytest.raises(ValueError, match="Unknown M5A slice IDs"):
        select_slices(config, ["not_a_slice"], None)
