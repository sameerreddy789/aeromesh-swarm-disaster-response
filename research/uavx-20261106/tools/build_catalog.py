"""Build an original index of the research ledgers, without copying source texts."""
from pathlib import Path
import json
import sys

sys.stdout.reconfigure(encoding="utf-8")
root = Path(__file__).resolve().parents[1]
files = ["root-sources.json", "physics-control-sources.json",
         "swarm-network-sources.json", "visual-demo-sources.json"]
rows, counts = [], {}
for name in files:
    data = json.loads((root / name).read_text(encoding="utf-8"))
    sources = data if isinstance(data, list) else data["sources"]
    counts[name] = len(sources)
    for source in sources:
        rows.append((name, source))
ids = [source["id"] for _, source in rows]
if len(ids) != len(set(ids)):
    raise SystemExit("Duplicate source ID")

text = ["# Research source catalog", "",
        f"{len(rows)} source records across four ledgers. Some references recur across specialties; this is not a count of independent studies or of papers read cover to cover.", "",
        "The ledgers retain inspected sections, access limits, supported claims and applicability. Search results and failed page retrievals are discovery records, not support for a claim. Full third-party source caches remain local and are excluded from publication.", "",
        "| Ledger | Records |", "|---|---:|"]
text += [f"| [{name}]({name}) | {count} |" for name, count in counts.items()]
text += ["", "## Indexed sources", ""]
for name, source in rows:
    sid = source["id"]
    url = source.get("url")
    title = source["title"]
    access = next((source.get(k) for k in ("reading_level", "content_access", "access") if source.get(k)), "See ledger")
    text += [f'<a id="{sid.lower()}"></a>', f"### {sid} — {title}", "",
             f"Source: [{title}]({url})" if url else f"Source: supplied local document, `{source.get('local_file', 'see ledger')}`",
             "", f"Reading scope: {access}", "", f"Claim details and limitations: [{name}]({name}).", ""]
(root / "SOURCE_CATALOG.md").write_text("\n".join(text), encoding="utf-8")
(root / "source-inventory.json").write_text(json.dumps({"records": len(rows), "ledgers": counts,
    "unique_ids": len(set(ids)), "count_is_independent_studies": False}, indent=2) + "\n", encoding="utf-8")
print(json.dumps(counts), "records", len(rows))
