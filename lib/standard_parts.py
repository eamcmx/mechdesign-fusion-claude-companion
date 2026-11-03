# -*- coding: utf-8 -*-
"""standard_parts.py: Chapter 50's work list. Every bought designation of the eight assemblies Volume 3 details
(detail_inventory.py), with its group, the assemblies that use it, its pieces, and the source of its data already in
hand: a maker's page saved in the references' Web pages folder, a table of Shigley, or a catalogue cited in Volume 2.
A designation with no source yet is marked so; Chapter 50 completes them.

    python standard_parts.py            -> the list, group by group, and the count still without a source
"""
import io
import os
import re

import detail_inventory as D

REFS = os.path.join(os.path.expanduser('~'), 'Documents', 'Mechanical Design References', 'Web pages')

# makers' pages saved in the references (file -> the designation it describes)
PAGES = {'SKF_32213_2026-11-01.md': 'Bearing 32213', 'SKF_32308_2026-11-01.md': 'Bearing 32308',
         'SKF_32309_2026-11-01.md': 'Bearing 32309', 'SKF_22207E_2026-11-02.md': 'Bearing 22207 E',
         'SKF_22213E_2026-11-02.md': 'Bearing 22213 E', 'SKF_61836_2026-11-02.md': 'Bearing 61836',
         'SKF_61856MA_2026-11-02.md': 'Bearing 61856 MA', 'SKF_6002-2RSH_2026-11-02.md': 'Bearing 6002-2RSH',
         'SKF_61902-2RS1_2026-11-02.md': 'Bearing 61902-2RS1', 'SKF_BSD2562CGA_2026-11-02.md': 'Bearing BSD 2562 CGA',
         'SKF_GE20ES_2026-11-02.md': 'Bearing GE 20 ES', 'SKF_SI20ES_2026-11-02.md': 'Rod end SI 20 ES',
         'SKF_slewing_bearings_2026-11-02.md': 'Slewing ring'}
# designations whose data come from a table or a catalogue Volume 2 cites (the source as the book cites it)
CITED = [
    (r'^Bearing 62\d\d$', 'Budynas & Nisbett (2015), Table 11-2, p. 573'),
    (r'^RV-\d+N$', 'Nabtesco Precision Europe GmbH (n.d.)'),
    (r'^Rail HGR|^Block HGH|^Ball ?screw|^Ball nut|^Fixed bearing BK|^Supported bearing BF', 'HIWIN GmbH (2017, 2022)'),
    (r'^Cylinder DSBC', 'Festo AG & Co. KG (2012)'),
    (r'^Cup SPB|^Ejector', 'J. Schmalz GmbH (n.d.)'),
    (r'^Belt HTD', 'Gates Corporation (n.d.)'),
    (r'^Spring$', 'designed to Budynas & Nisbett (2015), Chapter 10 (Chapter 46)'),
]


def page_data(fname):
    """The rows of a saved maker's page: {item: value}."""
    out = {}
    for line in io.open(os.path.join(REFS, fname), encoding='utf-8'):
        m = re.match(r'^\|\s*([^|]+?)\s*\|\s*([^|]+?)\s*\|\s*$', line)
        if m and not set(m.group(1)) <= set('-: ') and m.group(1) != 'Item':
            out[m.group(1)] = m.group(2)
    return out


def work_list(rows=None):
    rows = rows or D.all_machines()
    use, pieces, group = {}, {}, {}
    for r in rows:
        for g, dd in r['groups'].items():
            for d, n in dd.items():
                use.setdefault(d, []).append(r['key'])
                pieces[d] = pieces.get(d, 0) + n
                group[d] = g
    by_page = {v: k for k, v in PAGES.items()}
    out = []
    for d in sorted(use):
        if d in by_page and os.path.exists(os.path.join(REFS, by_page[d])):
            src = 'saved page ' + by_page[d]
        else:
            src = next((s for pat, s in CITED if re.search(pat, d)), None)
        out.append(dict(designation=d, group=group[d], uses=use[d], pieces=pieces[d], source=src))
    return out


if __name__ == '__main__':
    wl = work_list()
    for g in sorted({w['group'] for w in wl}):
        print('== %s' % g)
        for w in wl:
            if w['group'] == g:
                print('  %-40s %-22s %4d  %s' % (w['designation'], ','.join(w['uses']), w['pieces'], w['source'] or '-- none yet'))
    none = [w for w in wl if not w['source']]
    print('%d designations, %d with a source in hand, %d without' % (len(wl), len(wl) - len(none), len(none)))
