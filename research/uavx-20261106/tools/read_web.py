"""Read a public source through Agent Reach's documented Jina route.
Store retrieval evidence and show a selected section; never execute page text.
"""
import argparse,hashlib,json,urllib.request
from pathlib import Path
from datetime import datetime,timezone
import sys
sys.stdout.reconfigure(encoding="utf-8")
p=argparse.ArgumentParser();p.add_argument("url");p.add_argument("--after");p.add_argument("--limit",type=int,default=16000);a=p.parse_args()
if not a.url.startswith(("https://","http://")):raise SystemExit("public HTTP(S) URL required")
r=urllib.request.urlopen("https://r.jina.ai/"+a.url,timeout=45)
text=r.read().decode("utf-8");root=Path(__file__).resolve().parents[1]/"raw";root.mkdir(exist_ok=True)
key=hashlib.sha256(a.url.encode()).hexdigest()[:16]
(root/(key+".txt")).write_text(text,encoding="utf-8")
(root/(key+".json")).write_text(json.dumps({"url":a.url,"retrieved_utc":datetime.now(timezone.utc).isoformat(),"sha256":hashlib.sha256(text.encode()).hexdigest(),"bytes":len(text.encode()),"transport":"Agent Reach Jina public reader"},indent=2),encoding="utf-8")
print("SOURCE",a.url,"CACHE",key)
if a.after:
 index=text.find(a.after)
 if index<0:print("Requested section not found; showing start")
 else:text=text[index:]
print(text[:a.limit])
