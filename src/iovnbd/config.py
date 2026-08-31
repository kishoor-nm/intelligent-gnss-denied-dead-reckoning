"""
Configuration module for IO-VNBD dataset ingestion & inspection.
"""

import os
from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class DatasetConfig:
    dataset_root: str = "d:/prototype/IO-VNBD-master"
    output_dir: str = "d:/prototype/output_module1"
    selected_sequence: Optional[str] = "S1"
    selected_driver: Optional[str] = "S (Driver A)"
    subset_categorized: bool = True
    encoding_list: List[str] = field(default_factory=lambda: ["utf-8", "latin1", "cp1252", "iso-8859-1"])
    max_sample_records: Optional[int] = None

    def get_categorized_sync_path(self) -> str:
        return os.path.join(self.dataset_root, "Synchronised V abd S datasets", "Categorised IOVNB Dataset")

    def get_uncategorized_sync_path(self) -> str:
        return os.path.join(self.dataset_root, "Synchronised V abd S datasets", "Uncategorised IOVNB Dataset")

    def get_unsync_v_path(self) -> str:
        return os.path.join(self.dataset_root, "Unsynchronised V and S Dataset", "Categorised IOVNB (V) Dataset")
