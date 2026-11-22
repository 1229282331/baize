"""Check panel decisions independently of a display or serial hardware."""
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/panel"))
from panel_session import IMU_HEADER, WHEEL_HEADER, Wheel, library, observation_result, summarize, target_values
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from wheel_acceptance import replay


def recording(speed=1., progress=True):
    rows = []
    for i in range(25):
        active = 1 <= i <= 17
        angle = min(i, 17)*speed*.02 if progress else 0
        rows.append(Wheel(i*20, (2**32-80+i*20) % 2**32, int(active), 0 if active else 2, 12.4,
                          (angle, 0, 0, 0), (speed if active else 0, 0, 0, 0)))
    return rows


class PanelResults(unittest.TestCase):
    def result(self, rows, target=1., outcome="completed", settled=True):
        return summarize(rows, (target, 0, 0, 0), outcome, settled)

    def test_pass_across_board_tick_wrap(self):
        r = self.result(recording())
        self.assertEqual(r["assessment"], "反馈符合")
        self.assertEqual(r["wheels_match"], [True]*4)
        self.assertTrue(r["stop_ack"] and r["settled"])

    def test_incomplete_and_interrupted(self):
        self.assertEqual(self.result(recording()[:5])["assessment"], "样本不足")
        self.assertNotEqual(self.result(recording()[:18])["assessment"], "反馈符合")
        self.assertNotEqual(self.result(recording(), settled=False)["assessment"], "反馈符合")
        self.assertEqual(self.result(recording(), outcome="stopped")["assessment"], "手动停机")

    def test_bad_tracking_and_stalled_wheel(self):
        for rows, speed in [(recording(-1), 1), (recording(0), .01), (recording(progress=False), 1)]:
            self.assertFalse(self.result(rows, speed)["wheels_match"][0])
        rows = recording()
        rows[10] = rows[10]._replace(angle=(.20, .1, 0, 0))
        self.assertFalse(self.result(rows)["wheels_match"][1])

    def test_feedback_gap_or_interrupted_enabled_interval(self):
        for change in (dict(host=600), dict(board=9000), dict(state=0, reason=2)):
            rows = recording()
            rows[6] = rows[6]._replace(**change)
            self.assertNotEqual(self.result(rows)["assessment"], "反馈符合")

    def test_operator_inputs(self):
        self.assertEqual(target_values("单轮", "右后", "反向", "", "1.5", (), ".8")[1:],
                         (800, (0, 0, 0, -1.5)))
        self.assertEqual(target_values("整车", "", "", "左转", ".5", (), "2")[2], (-.5, .5, -.5, .5))
        for speed, duration in (("nan", "1"), ("2.01", "1"), (".0001", "2"), ("1", "inf"), ("1", "2.001")):
            with self.assertRaises(ValueError):
                target_values("单轮", "左前", "正向", "", speed, (), duration)
        custom = (1, -.3, .7, -2)
        self.assertEqual(target_values("四轮自定", "", "", "", "", custom, "2")[2], custom)

    def test_independent_physical_observation(self):
        target = (0, -.7, 0, 0)
        self.assertEqual(observation_result(target, "右前", "朝车尾"), "一致")
        self.assertEqual(observation_result(target, "右前", "朝车头"), "不符")
        self.assertEqual(observation_result(target, "左前", ""), "不符")
        self.assertEqual(observation_result(target, "右前", "未看清"), "待核对")
        self.assertEqual(observation_result((1, 1, 1, 1), "多轮", "朝车头"), "已记录")

    def test_replay_checks_raw_record_against_saved_judgment(self):
        rows = recording()
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with (folder/"wheels.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(WHEEL_HEADER)
                for i, r in enumerate(rows):
                    w.writerow([r.host, r.board, r.state, r.reason, r.volts, *r.angle, *r.speed,
                                "trial-001" if i else ""])
            with (folder/"imu.csv").open("w", newline="") as f:
                csv.writer(f).writerow(IMU_HEADER)
            saved = dict(id="trial-001", label="fl+", target_rad_s=[1., 0, 0, 0],
                         start_board_ms=rows[0].board, **self.result(rows))
            path = folder/"trial-001.json"
            path.write_text(json.dumps(saved), encoding="utf-8")
            self.assertEqual(replay(folder)["trials"][0]["assessment"], "反馈符合")
            saved["mean_rad_s"][0] = 1.5
            path.write_text(json.dumps(saved), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "mean_rad_s"):
                replay(folder)


if __name__ == "__main__":
    library(sys.argv.pop(1))
    unittest.main()
