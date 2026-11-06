"""Create one static plot from the recorded radio sensitivity rows."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

ROOT=Path(__file__).resolve().parent
rows=[json.loads(x) for x in (ROOT/"cases.jsonl").read_text().splitlines()]
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10})
fig, axes=plt.subplots(1,3,figsize=(12,4.1),sharey=True)
for ax,size,title in zip(axes,(512,32768,262144),("512 B","32 KiB","256 KiB")):
    labels=[]; values=[]
    for dt in (.5,.25,.1):
        rr=[]; vv=[]
        for hops in (1,5,10,14):
            r=next(r for r in rows if r["dt_s"]==dt and r["hops"]==hops and
                   r["raw_frame_bytes"]==size and r["iid_loss_per_attempt"]==0
                   and r["mode"]=="single_offer" and not r["scripted_first_offer_chunk_loss"])
            v=r["image_complete_s"]
            rr.append(">15" if v is None else f"{v:g}")
            vv.append(0 if v is not None and v<=10 else 1)
        labels.append(rr);values.append(vv)
    ax.imshow(np.array(values),cmap=ListedColormap(["#d8ece8","#f6dcc1"]),vmin=0,vmax=1,aspect="auto")
    for i in range(3):
        for j in range(4):ax.text(j,i,labels[i][j],ha="center",va="center",fontsize=13,color="#172d34")
    ax.set_title(title,fontsize=13,pad=12,fontweight="bold")
    ax.set_xticks(range(4),[1,5,10,14]);ax.set_xlabel("Path hops")
    ax.set_yticks(range(3),["0.50 s","0.25 s","0.10 s"])
    ax.set_xticks(np.arange(-.5,4,1),minor=True);ax.set_yticks(np.arange(-.5,3,1),minor=True)
    ax.grid(which="minor",color="white",linewidth=3);ax.tick_params(which="minor",bottom=False,left=False)
    for s in ax.spines.values():s.set_visible(False)
axes[0].set_ylabel("Forwarding step / hop delay")
fig.suptitle("Complete image receipt time (seconds), zero-loss isolated chains",fontsize=15,fontweight="bold",x=.5,y=.98)
fig.text(.5,.085,"Green: within 10 s     Orange: misses 10 s     >15: unfinished at 15 s cutoff",ha="center",fontsize=10)
fig.text(.5,.03,"Single application offer · 128 KiB/s shared TX/RX per node · 1 MiB queues · No background traffic or flight",ha="center",fontsize=9,color="#4c5b61")
fig.tight_layout(rect=(0,.13,1,.9))
fig.savefig(ROOT/"zero-loss-completion.png",dpi=180)
fig.savefig(ROOT/"zero-loss-completion.svg")

