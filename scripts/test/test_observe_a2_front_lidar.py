"""用合成 PointCloud2 样本验证前雷达只读诊断，无需安装 ROS。"""

import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from observe_a2_front_lidar import summarize_cloud


def make_cloud(rows, *, big_endian=False, padding=b""):
    # xyz 字段顺序和存储位置刻意打乱，靠字段描述确定每个分量。
    width = len(rows[0])
    fields = [
        SimpleNamespace(name="z", offset=4, datatype=7, count=1),
        SimpleNamespace(name="x", offset=8, datatype=7, count=1),
        SimpleNamespace(name="y", offset=0, datatype=7, count=1),
        SimpleNamespace(name="intensity", offset=12, datatype=7, count=1),
    ]
    pack = struct.Struct(">ffff" if big_endian else "<ffff").pack
    data = b"".join(b"".join(pack(y, z, x, 0.5) for x, y, z in row) + padding for row in rows)
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="hesai_lidar"),
        fields=fields,
        width=width,
        height=len(rows),
        point_step=16,
        row_step=16 * width + len(padding),
        data=data,
        is_bigendian=big_endian,
    )


class FrontCloudTests(unittest.TestCase):
    def test_closest_return_uses_only_forward_raw_lidar_section(self):
        cloud = make_cloud([[
            (-0.25, 0.0625, 4.5),
            (0.25, -0.0625, 2.25),
            (0.0, 0.0, -0.125),
            (0.375, 0.0, 0.125),
            (0.0, 0.125, 0.25),
            (0.0, 0.0, 0.0),
            (math.nan, 0.0, 0.125),
            (0.0, math.inf, 0.125),
            (0.0, 0.0, math.nan),
        ]])
        # 后方、原点和前向截面之外的有限点只计入总点数。
        self.assertEqual(summarize_cloud(cloud), (2.25, 6))

    def test_big_endian_rows_ignore_padding_that_looks_like_a_near_return(self):
        extra = struct.pack(">ffff", 0.0, 0.125, 0.0, 0.0)
        cloud = make_cloud([[(0.0, 0.0, 4.5), (0.0, 0.0, 5.0)],
                            [(0.0, 0.0, 1.75), (0.75, 0.0, 0.5)]],
                           big_endian=True, padding=extra)
        self.assertEqual(summarize_cloud(cloud), (1.75, 4))

    def test_missing_return_reports_unknown_and_counts_other_valid_points(self):
        out_of_section = make_cloud([[(0.5, 0.0, 4.5), (0.0, 0.0, -1.0)]])
        no_finite_point = make_cloud([[(math.nan, 0.0, 4.5)]])
        self.assertEqual(summarize_cloud(out_of_section), (None, 2))
        self.assertEqual(summarize_cloud(no_finite_point), (None, 0))

    def test_wrong_frame_and_malformed_layout_are_rejected(self):
        def wrong_frame(cloud):
            cloud.header.frame_id = "base_link"

        def truncate(cloud):
            cloud.data = cloud.data[:-1]

        def undersized_row(cloud):
            cloud.row_step -= 1

        def missing_axis(cloud):
            cloud.fields.pop(0)

        def wide_datatype(cloud):
            cloud.fields[0].datatype = 8

        def vector_axis(cloud):
            cloud.fields[0].count = 2

        def crossing_offset(cloud):
            cloud.fields[0].offset = cloud.point_step - 3

        def duplicate_axis(cloud):
            cloud.fields[-1].name = "x"

        def overlapping_axes(cloud):
            cloud.fields[0].offset = cloud.fields[1].offset - 1

        # 每个案例只破坏一项，避免因前一个错误掩盖后续字段检查。
        for corrupt in (wrong_frame, truncate, undersized_row, missing_axis, wide_datatype,
                        vector_axis, crossing_offset, duplicate_axis, overlapping_axes):
            with self.subTest(case=corrupt.__name__):
                cloud = make_cloud([[(0.0, 0.0, 4.5)]])
                corrupt(cloud)
                with self.assertRaises(ValueError):
                    summarize_cloud(cloud)


if __name__ == "__main__":
    unittest.main()
