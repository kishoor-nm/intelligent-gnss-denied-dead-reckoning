"""
Dataset inventory builder (machine-readable JSON & human-readable Markdown).
"""

import os
import json
from dataclasses import asdict
from typing import Dict, Any, List

def generate_inventory_json(inventory_data: Dict[str, Any], output_path: str):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(inventory_data, f, indent=2, default=str)

def generate_inventory_markdown(inventory_data: Dict[str, Any], output_path: str):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    md = []
    md.append(f"# IO-VNBD Dataset Inventory Report")
    md.append(f"**Dataset Name**: {inventory_data.get('dataset_name', 'IO-VNBD')}")
    md.append(f"**Root Directory**: `{inventory_data.get('root_directory')}`")
    md.append(f"**Inspection Status**: {inventory_data.get('inspection_status')}\n")

    md.append("## Sequences Discovered")
    for seq in inventory_data.get("sequences", []):
        md.append(f"### Sequence: {seq.get('sequence_name')}")
        md.append(f"- **Driver / Category**: {seq.get('driver')}")
        md.append(f"- **Path**: `{seq.get('path')}`")
        for st in seq.get("streams", []):
            md.append(f"  - **Stream**: `{st.get('file_name')}`")
            md.append(f"    - Status: `{st.get('status')}`")
            md.append(f"    - Records: {st.get('record_count', 0)}")
            md.append(f"    - Effective Freq: {st.get('effective_hz', 'N/A')} Hz (Documented: {st.get('documented_hz', 'N/A')} Hz)")
            md.append(f"    - Schema Match: {st.get('schema_exact_match', False)}")

    md.append("\n## Data Stream Schemas & Fields")
    for stream_schema in inventory_data.get("schemas", []):
        md.append(f"### {stream_schema.get('name')}")
        md.append(f"Total Fields: {stream_schema.get('column_count')}")
        md.append("```")
        for col in stream_schema.get("columns", []):
            md.append(f"- {col}")
        md.append("```\n")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md))
