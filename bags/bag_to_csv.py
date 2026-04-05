#!/usr/bin/env python3
import argparse
import csv
import os
import subprocess
import tempfile
from collections import OrderedDict
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def safe_topic_name(topic_name: str) -> str:
    return topic_name.strip("/").replace("/", "__") or "root"


def flatten_value(prefix, value, row):
    if hasattr(value, "get_fields_and_field_types"):
        for field_name in value.get_fields_and_field_types().keys():
            child = getattr(value, field_name)
            child_prefix = f"{prefix}_{field_name}" if prefix else field_name
            flatten_value(child_prefix, child, row)
        return

    if not isinstance(value, (str, bytes, bytearray)):
        try:
            iter(value)
            is_iterable = True
        except TypeError:
            is_iterable = False

        if is_iterable:
            value_list = list(value)
            if value_list and hasattr(value_list[0], "get_fields_and_field_types"):
                for idx, item in enumerate(value_list):
                    flatten_value(f"{prefix}_{idx}", item, row)
            else:
                for idx, item in enumerate(value_list):
                    row[f"{prefix}_{idx}"] = item
            return

    row[prefix] = value


def message_to_row(msg, stamp_ns, topic_name):
    row = OrderedDict()
    row["stamp_ns"] = stamp_ns
    row["t_sec"] = stamp_ns * 1e-9
    row["topic_name"] = topic_name
    prefix = safe_topic_name(topic_name) + "__"

    if hasattr(msg, "get_fields_and_field_types"):
        for field_name in msg.get_fields_and_field_types().keys():
            flatten_value(prefix + field_name, getattr(msg, field_name), row)
    else:
        row[prefix + "data"] = msg

    return row


def resolve_bag_dir(bag_arg: str) -> Path:
    script_dir = Path(__file__).resolve().parent

    if bag_arg == "latest":
        candidates = sorted(
            [p for p in script_dir.glob("bag_all_*") if p.is_dir()],
            key=lambda p: p.stat().st_mtime,
        )
        if not candidates:
            raise FileNotFoundError(f"No bag directories found in {script_dir}")
        return candidates[-1]

    candidate = Path(bag_arg)
    if candidate.is_dir():
        return candidate.resolve()

    candidate_in_bags = script_dir / bag_arg
    if candidate_in_bags.is_dir():
        return candidate_in_bags.resolve()

    raise FileNotFoundError(
        f"Bag directory not found: '{bag_arg}'. "
        f"Tried '{candidate}' and '{candidate_in_bags}'."
    )


def prepare_readable_bag_dir(bag_dir: Path):
    metadata_path = bag_dir / "metadata.yaml"
    with open(metadata_path, "r") as f:
        metadata = yaml.safe_load(f)

    info = metadata["rosbag2_bagfile_information"]
    compression_mode = info.get("compression_mode", "")
    compression_format = info.get("compression_format", "")

    if compression_mode != "FILE" or compression_format != "zstd":
        return bag_dir, None

    temp_dir = tempfile.TemporaryDirectory(prefix="bag_decompressed_")
    temp_bag_dir = Path(temp_dir.name) / bag_dir.name
    temp_bag_dir.mkdir(parents=True, exist_ok=True)

    for relpath in info.get("relative_file_paths", []):
        src = bag_dir / relpath
        if src.suffix == ".zstd":
            dest = temp_bag_dir / src.stem
            subprocess.run(
                ["zstd", "-d", "-f", str(src), "-o", str(dest)],
                check=True,
            )
        else:
            raise RuntimeError(f"Unsupported compressed file entry: {src}")

    new_metadata = dict(metadata)
    new_info = dict(info)
    new_info["compression_mode"] = ""
    new_info["compression_format"] = ""
    new_info["relative_file_paths"] = [
        Path(p).stem if str(p).endswith(".zstd") else p
        for p in info.get("relative_file_paths", [])
    ]
    new_metadata["rosbag2_bagfile_information"] = new_info

    with open(temp_bag_dir / "metadata.yaml", "w") as f:
        yaml.safe_dump(new_metadata, f, sort_keys=False)

    return temp_bag_dir, temp_dir


class CsvWriter:
    def __init__(self, csv_path, initial_fields):
        self.csv_path = csv_path
        self.rows = []
        self.fieldnames = list(initial_fields)
        self.rows.append(dict(initial_fields))

    def add_row(self, row):
        missing = [key for key in row.keys() if key not in self.fieldnames]
        if missing:
            self.fieldnames.extend(missing)
        self.rows.append(dict(row))

    def close(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Convert a ROS 2 bag to per-topic CSV files.")
    parser.add_argument(
        "bag_dir",
        help="Path to rosbag2 directory, bag folder name, or 'latest'",
    )
    parser.add_argument(
        "--out-dir",
        default="csv_out",
        help="Directory where the merged CSV file will be written",
    )
    args = parser.parse_args()

    bag_dir = resolve_bag_dir(args.bag_dir)
    readable_bag_dir, temp_dir = prepare_readable_bag_dir(bag_dir)

    out_dir = Path(args.out_dir)
    if not out_dir.is_absolute():
        out_dir = Path.cwd() / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(
        uri=str(readable_bag_dir),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    merged_csv_path = out_dir / "all_topics.csv"
    writer = None

    while reader.has_next():
        topic_name, serialized_data, stamp_ns = reader.read_next()
        msg_type = get_message(type_map[topic_name])
        msg = deserialize_message(serialized_data, msg_type)
        row = message_to_row(msg, stamp_ns, topic_name)

        if writer is None:
            writer = CsvWriter(str(merged_csv_path), row)
        else:
            writer.add_row(row)

    if writer is not None:
        writer.close()

    print(f"Bag: {bag_dir}")
    if temp_dir is not None:
        print(f"Decompressed temp bag: {readable_bag_dir}")
    print(f"Wrote merged CSV to {merged_csv_path}")


if __name__ == "__main__":
    main()
