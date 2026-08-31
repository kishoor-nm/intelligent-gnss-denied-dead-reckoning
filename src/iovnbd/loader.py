"""
Reusable loader for IO-VNBD dataset files and LFS detection.
"""

import os
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple, List
import pandas as pd

@dataclass
class FileStatus:
    file_path: str
    exists: bool
    is_lfs_pointer: bool
    file_size_bytes: int
    lfs_oid: Optional[str] = None
    lfs_size: Optional[int] = None
    error: Optional[str] = None

@dataclass
class LoadedStream:
    file_status: FileStatus
    dataframe: Optional[pd.DataFrame] = None
    encoding_used: Optional[str] = None
    record_count: int = 0
    schema_info: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

def check_file_status(file_path: str) -> FileStatus:
    if not os.path.exists(file_path):
        return FileStatus(file_path=file_path, exists=False, is_lfs_pointer=False, file_size_bytes=0, error="File does not exist")
    
    size = os.path.getsize(file_path)
    if size < 500:
        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                content = f.read()
                if "version https://git-lfs.github.com/spec/v1" in content:
                    lines = content.splitlines()
                    oid = None
                    lfs_size = None
                    for line in lines:
                        if line.startswith("oid sha256:"):
                            oid = line.split(":")[-1].strip()
                        elif line.startswith("size "):
                            lfs_size = int(line.split()[-1].strip())
                    return FileStatus(
                        file_path=file_path,
                        exists=True,
                        is_lfs_pointer=True,
                        file_size_bytes=size,
                        lfs_oid=oid,
                        lfs_size=lfs_size
                    )
        except Exception as e:
            pass

    return FileStatus(file_path=file_path, exists=True, is_lfs_pointer=False, file_size_bytes=size)

def load_iovnbd_csv(file_path: str, encodings: List[str] = ["utf-8", "latin1", "cp1252", "iso-8859-1"]) -> LoadedStream:
    status = check_file_status(file_path)
    if not status.exists:
        return LoadedStream(file_status=status, error=f"File not found: {file_path}")
    
    if status.is_lfs_pointer:
        return LoadedStream(
            file_status=status,
            error=f"File is Git LFS pointer object (OID: {status.lfs_oid}, Target size: {status.lfs_size} bytes). Real payload not downloaded locally."
        )

    last_error = None
    for enc in encodings:
        try:
            df = pd.read_csv(file_path, encoding=enc)
            # Strip leading/trailing whitespace from column names to handle raw header inconsistencies cleanly
            df.columns = df.columns.str.strip()
            return LoadedStream(
                file_status=status,
                dataframe=df,
                encoding_used=enc,
                record_count=len(df)
            )
        except Exception as e:
            last_error = str(e)
            continue
            
    return LoadedStream(file_status=status, error=f"Failed to read CSV with encodings {encodings}: {last_error}")
