# -*- coding: utf-8 -*-
"""Write each prompt's last FUSION CONVENTIONS line: its check and the names the check looks for (tools/check_names.py).

    python tools/put_check_lines.py          every printed prompt; a line already there is replaced

Run it again after a check changes the names it calls, then `python tools/check_names.py --all`.
"""
import io
import os
import sys
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_names as C  # noqa: E402

HEAD = "- The companion's check,"


def put(name):
    n = C.names(name)
    p = os.path.join(os.path.dirname(C.script_for(name)), 'PROMPT.txt')
    lines = io.open(p, encoding='utf-8').read().split('\n')
    # drop a line written before, with its continuation lines
    out, skip = [], False
    for ln in lines:
        if ln.startswith(HEAD):
            skip = True
            continue
        if skip and ln.startswith('  '):
            continue
        skip = False
        out.append(ln)
    i0 = next(i for i, ln in enumerate(out) if ln.strip().upper().startswith('FUSION CONVENTIONS'))
    i1 = next(i for i, ln in enumerate(out) if i > i0 and ln.strip().upper().startswith('ACCEPTANCE CRITERIA'))
    j = i1
    while j > i0 and not out[j - 1].strip():
        j -= 1
    new = textwrap.wrap(C.line(n), width=118, subsequent_indent='  ', break_long_words=False, break_on_hyphens=False)
    out[j:j] = new
    io.open(p, 'w', encoding='utf-8', newline='\n').write('\n'.join(out))
    return p


if __name__ == '__main__':
    for nm in C.prompts():
        print(os.path.relpath(put(nm), C.ROOT))
