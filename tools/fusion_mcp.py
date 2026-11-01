# -*- coding: utf-8 -*-
"""Talk to desktop Fusion through the local endpoint it announces in its Text Commands palette
("MCP - http://127.0.0.1:27183/mcp", Fusion 2705 and later). No mouse, no screen.

    python fusion_mcp.py script <file.py> [--readonly] [--out result.txt]
        runs the file in Fusion. The file must define run(_context) and print what it found;
        it must NOT catch its own exceptions (Fusion returns the traceback).
    python fusion_mcp.py shot <out.png> [direction] [width height]
        saves a picture of the graphics view. direction: current, front, back, top, bottom, left,
        right, iso-top-right, iso-top-left, iso-bottom-right, iso-bottom-left
    python fusion_mcp.py read '<json arguments of fusion_mcp_read>'
    python fusion_mcp.py exec '<json arguments of fusion_mcp_execute>'

What it cannot do: the Simulation workspace has no API. Studies, loads, contacts and solving are still set
by hand in the window. Measuring, masses, pictures and anything in the Design workspace go through here.
Fusion must be running with a document open. Saving a document is never done from here.
"""
import base64
import io
import json
import sys
import urllib.request

sys.stdout.reconfigure(encoding="utf-8", errors="replace")
URL = "http://127.0.0.1:27183/mcp"
HEAD = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}


def post(payload, session=None, timeout=600):
    h = dict(HEAD)
    if session:
        h["MCP-Session-Id"] = session
    req = urllib.request.Request(URL, data=json.dumps(payload).encode("utf-8"), headers=h, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read().decode("utf-8", "replace")
        sid = r.headers.get("MCP-Session-Id")
    if body.lstrip().startswith("event:") or body.lstrip().startswith("data:"):
        data = [line[5:].strip() for line in body.splitlines() if line.startswith("data:")]
        body = data[-1] if data else ""
    return (json.loads(body) if body.strip() else None), sid


def call(tool, arguments, timeout=600):
    init, sid = post({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                      "params": {"protocolVersion": "2025-03-26", "capabilities": {}, "clientInfo": {"name": "rdm-workshop", "version": "1"}}})
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, sid)
    out, _ = post({"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": tool, "arguments": arguments}}, sid, timeout)
    return out


def texts(out):
    if out is None:
        return ""
    if "error" in out:
        return "ERROR " + json.dumps(out["error"])
    res = out.get("result", {})
    parts = []
    for c in res.get("content", []):
        if c.get("type") == "text":
            parts.append(c["text"])
        elif c.get("type") == "image":
            parts.append("<image %s, %d base64 characters>" % (c.get("mimeType"), len(c.get("data", ""))))
    return ("TOOL ERROR\n" if res.get("isError") else "") + "\n".join(parts)


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    cmd = argv[0]
    if cmd == "script":
        src = io.open(argv[1], encoding="utf-8").read()
        obj = {"script": src}
        if "--readonly" in argv:
            obj["readOnly"] = True
        t = texts(call("fusion_mcp_execute", {"featureType": "script", "object": obj}))
        if "--out" in argv:
            p = argv[argv.index("--out") + 1]
            io.open(p, "w", encoding="utf-8", newline="\n").write(t)
            print("written %s (%d characters)" % (p, len(t)))
            print(t[:600])
        else:
            print(t)
        return 1 if t.startswith(("ERROR", "TOOL ERROR")) else 0
    if cmd == "shot":
        args = {"queryType": "screenshot", "transparentBackground": False}
        if len(argv) > 2:
            args["direction"] = argv[2]
        if len(argv) > 4:
            args["width"], args["height"] = int(argv[3]), int(argv[4])
        out = call("fusion_mcp_read", args)
        res = (out or {}).get("result", {})
        for c in res.get("content", []):
            data = c.get("data")
            if c.get("type") == "text":
                try:
                    j = json.loads(c["text"])
                    data = j.get("base64Data") or data
                except Exception:
                    pass
            if data:
                raw = base64.b64decode(data)
                open(argv[1], "wb").write(raw)
                print("saved %s (%d bytes)" % (argv[1], len(raw)))
                return 0
        print("no image came back:", texts(out)[:800])
        return 1
    if cmd in ("read", "exec"):
        print(texts(call("fusion_mcp_read" if cmd == "read" else "fusion_mcp_execute", json.loads(argv[1]))))
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
