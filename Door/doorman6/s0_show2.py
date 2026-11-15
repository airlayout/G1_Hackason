import json,sys,collections
rows=[json.loads(l) for l in open(sys.argv[1])]
first=[d for d in rows if d["episode"]==0]
print("各台の 1 回目",len(first))
def grp(name,sel):
    g=[d for d in first if sel(d)]
    if not g: return
    c=collections.Counter(d["last_sub"] for d in g)
    r=collections.Counter((d["stop"].split("（")[0])[:34] for d in g)
    print(f"{name}: {len(g)} 台・レバー 30° {sum(d['lever_max_deg']>=30 for d in g)}・扉 3° {sum(d['door_max_deg']>=3 for d in g)}・最後 {dict(c)}")
    for k,v in r.most_common(6): print("     ",v,k)
grp("全部",lambda d:True)
grp("レバーの高さ 0.95 m 未満",lambda d:d["lever_height"]<0.95)
grp("レバーの高さ 0.95 m 以上",lambda d:d["lever_height"]>=0.95)
if "-v" in sys.argv:
    for d in first: print(d["env"],round(d["lever_height"],2),round(d["door_width"],2),d.get("start"),[s[0]+"@"+str(s[1]) for s in d["subs"]],"lev",round(d["lever_max_deg"]),"door",round(d["door_max_deg"],1),d["stop"][:44],d.get("palm"))
