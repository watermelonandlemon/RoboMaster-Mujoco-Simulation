# -*- coding: utf-8 -*-
from pathlib import Path
import datetime
import json
import re
import shutil
import tkinter as tk
from tkinter import ttk, messagebox
import xml.etree.ElementTree as ET
import mujoco
import mujoco.viewer

HERE = Path(__file__).resolve().parent
SCENE = HERE / 'scene_color.xml'
NOTES = HERE / 'field_mesh_notes.json'
EXPORT = HERE / 'scene_color_collision_selected.xml'
NAME_RE = re.compile(r'^field_geom_\d{3}$')
TAG_RE = re.compile(r'<geom\b[^>]*\bname="(field_geom_\d{3})"[^>]*/>', re.S)


def bounds_of_obj(path):
    lo = [float('inf')]*3
    hi = [float('-inf')]*3
    nvertex = nface = 0
    with path.open('r', encoding='utf-8', errors='replace') as f:
        for line in f:
            if line.startswith('v '):
                xyz = [float(x) for x in line.split()[1:4]]
                if len(xyz) != 3: continue
                nvertex += 1
                for k in range(3):
                    lo[k] = min(lo[k], xyz[k]); hi[k] = max(hi[k], xyz[k])
            elif line.startswith('f '):
                nface += 1
    return lo, hi, nvertex, nface


def field_items(model):
    xml = ET.parse(SCENE).getroot()
    compiler = xml.find('compiler')
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    meshes = {e.get('name'): e.get('file') for e in xml.findall('asset/mesh')}
    items = []
    for geom in xml.findall('worldbody/geom'):
        name = geom.get('name', '')
        if not NAME_RE.fullmatch(name): continue
        mesh = meshes[geom.get('mesh')]
        filename = HERE / meshdir / mesh
        lo, hi, nv, nf = bounds_of_obj(filename)
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0: raise ValueError(f'无法在 MuJoCo 中找到 {name}')
        color = [float(x) for x in geom.get('rgba', '1 1 1 1').split()]
        enabled = geom.get('contype', '1') != '0' and geom.get('conaffinity', '1') != '0'
        items.append(dict(name=name, gid=gid, file=filename.name, lo=lo, hi=hi,
                          nv=nv, nf=nf, color=color, enabled=enabled, label=''))
    items.sort(key=lambda i: i['name'])
    if NOTES.exists():
        try:
            saved = json.loads(NOTES.read_text(encoding='utf-8'))
            for i in items:
                old = saved.get(i['name'], {})
                i['label'] = old.get('label', '')
                i['enabled'] = bool(old.get('collision', i['enabled']))
        except (OSError, ValueError): pass
    return items


def patched_xml(items):
    byname = {i['name']:i for i in items}
    found = set()
    def patch(m):
        name = m.group(1)
        if name not in byname: return m.group(0)
        found.add(name)
        flag = '1' if byname[name]['enabled'] else '0'
        tag = m.group(0)
        for attr in ('contype','conaffinity'):
            pattern = rf'\b{attr}="[^"]*"'
            if re.search(pattern,tag):
                tag = re.sub(pattern,f'{attr}="{flag}"',tag,count=1)
            else:
                tag = tag[:-2] + f' {attr}="{flag}"/>'
        return tag
    result = TAG_RE.sub(patch, SCENE.read_bytes().decode('utf-8-sig'))
    if found != set(byname):
        raise ValueError('未能匹配全部场地模型：' + repr(sorted(set(byname)-found)))
    ET.fromstring(result)
    return result


def main():
    if not SCENE.exists():
        raise SystemExit('请把 field_mesh_inspector.py 放在 scene_color.xml 同级目录。')
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    items = field_items(model)
    byname = {i['name']: i for i in items}
    original = model.geom_rgba.copy()
    ids = [i['gid'] for i in items]
    field_idset = set(ids)

    root = tk.Tk()
    root.title('RMUC2027 场地模型识别器 · 42 个 OBJ')
    root.geometry('1020x710')
    root.minsize(850, 580)
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
    except Exception:
        root.destroy()
        raise

    root.rowconfigure(1, weight=1)
    root.columnconfigure(0, weight=1)
    ttk.Label(root, text='选择模型编号 → 三维场景高亮 → 填写部件名称 → 按需勾选碰撞。'
                         '注意：按颜色合并的 OBJ 可能同时包含很多不同物体。',
              wraplength=980).grid(row=0, column=0, sticky='w', padx=12, pady=10)

    paned = ttk.Panedwindow(root, orient='horizontal')
    paned.grid(row=1, column=0, sticky='nsew', padx=12)
    left = ttk.Frame(paned)
    right = ttk.Frame(paned, padding=(12,0,0,0))
    paned.add(left, weight=3)
    paned.add(right, weight=2)
    left.rowconfigure(0, weight=1)
    left.columnconfigure(0, weight=1)
    tree = ttk.Treeview(left, columns=('rgb','collision','label'), show='tree headings', selectmode='browse')
    for key,text,width in [('#0','网格编号',155), ('rgb','颜色',90),
                           ('collision','碰撞',60), ('label','人工标注',145)]:
        tree.heading(key, text=text)
        tree.column(key, width=width, anchor='w', stretch=(key=='label'))
    scroll = ttk.Scrollbar(left, command=tree.yview)
    tree.configure(yscrollcommand=scroll.set)
    tree.grid(row=0, column=0, sticky='nsew')
    scroll.grid(row=0, column=1, sticky='ns')
    def colorcode(item):
        return '#' + ''.join(f'{max(0,min(255,round(x*255))):02X}' for x in item['color'][:3])
    for item in items:
        tree.insert('', 'end', iid=item['name'], text=item['name'],
                    values=(colorcode(item),'开' if item['enabled'] else '关',item['label']))

    selected = None
    mode = tk.StringVar(value='ghost')
    collision = tk.BooleanVar(value=False)
    note = tk.StringVar()
    info = tk.StringVar()
    location = tk.StringVar()
    warning = tk.StringVar()
    status = tk.StringVar(value='当前仅进行静态预览，不运行飞镖动力学仿真。')
    ttk.Label(right, text='当前网格', font=('Microsoft YaHei UI',13,'bold')).pack(anchor='w', pady=(0,10))
    ttk.Label(right, textvariable=info, justify='left', wraplength=330).pack(anchor='w', pady=4)
    ttk.Label(right, textvariable=location, justify='left', wraplength=330).pack(anchor='w', pady=4)
    ttk.Label(right, textvariable=warning, justify='left', wraplength=330).pack(anchor='w', pady=4)
    ttk.Separator(right).pack(fill='x', pady=12)
    ttk.Label(right, text='三维预览方式').pack(anchor='w')
    for title,value in [('选中高亮、其他半透明','ghost'),
                        ('仅显示选中的网格','only'),
                        ('恢复所有原始颜色','original')]:
        ttk.Radiobutton(right,text=title,variable=mode,value=value).pack(anchor='w',pady=2)
    ttk.Separator(right).pack(fill='x', pady=12)
    ttk.Label(right, text='这组网格对应哪些结构？').pack(anchor='w')
    entry = ttk.Entry(right,textvariable=note)
    entry.pack(fill='x',pady=6)
    ttk.Checkbutton(right,text='该网格参与碰撞（谨慎开启）',variable=collision).pack(anchor='w',pady=6)

    def update_row():
        if selected is None: return
        item = byname[selected]
        item['label'] = note.get().strip()
        item['enabled'] = bool(collision.get())
        tree.set(selected,'label',item['label'])
        tree.set(selected,'collision','开' if item['enabled'] else '关')

    def recolor(*_):
        model.geom_rgba[:] = original
        if mode.get() != 'original':
            for gid in ids:
                model.geom_rgba[gid,3] = 0.10 if mode.get()=='ghost' else 0.0
            if selected:
                model.geom_rgba[byname[selected]['gid'],:] = (1.0,0.15,0.55,1.0)
        # 隐藏飞镖与临时地面，防止它们遮挡场地模型
        for gid in range(model.ngeom):
            if gid not in field_idset:
                model.geom_rgba[gid,3] = 0.0
        if viewer.is_running(): viewer.sync()

    def focus():
        if not selected or not viewer.is_running(): return
        item = byname[selected]
        lo,hi=item['lo'],item['hi']
        extent=max(hi[k]-lo[k] for k in range(3))
        with viewer.lock():
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = [(lo[k]+hi[k])/2 for k in range(3)]
            viewer.cam.distance = max(1.5,1.7*extent)
            viewer.cam.azimuth = 150
            viewer.cam.elevation = -50
        viewer.sync()

    def on_selection(_event=None):
        nonlocal selected
        new = tree.selection()
        if not new or new[0]==selected: return
        update_row()
        selected = new[0]
        item = byname[selected]
        lo,hi=item['lo'],item['hi']
        info.set(f"{selected}\n{item['file']}\n颜色 {colorcode(item)}\n"
                 f"顶点 {item['nv']:,} / 三角面 {item['nf']:,}")
        location.set('XYZ最小: ' + ', '.join(f'{v:.2f}' for v in lo) + ' m\n'
                     'XYZ最大: ' + ', '.join(f'{v:.2f}' for v in hi) + ' m\n'
                     '包围尺寸: ' + ' × '.join(f'{hi[k]-lo[k]:.2f}' for k in range(3)) + ' m')
        spread=max(hi[0]-lo[0],hi[1]-lo[1])
        warning.set('⚠ 分布跨度大：同一 OBJ 很可能涉及多处不同结构。' if spread>10
                    else '提醒：同色 OBJ 不一定是一件独立物体。')
        note.set(item['label']);collision.set(item['enabled'])
        recolor();focus()

    def move(offset):
        idx=next((k for k,v in enumerate(items) if v['name']==selected),0)
        name=items[(idx+offset)%len(items)]['name']
        tree.selection_set(name);tree.see(name);tree.focus(name)
        on_selection()

    def save_notes():
        update_row()
        obj={item['name']:{'label':item['label'],'collision':item['enabled'],
                           'mesh':item['file']} for item in items}
        NOTES.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
        status.set(f'已保存：{NOTES.name}')

    def save_xml(overwrite=False):
        save_notes()
        try:
            content=patched_xml(items)
            n=sum(i['enabled'] for i in items)
            if n and not messagebox.askyesno('确认 OBJ 碰撞设置',
                  f'你选择开启 {n} 个颜色分组 OBJ 的碰撞。复杂凹形网格在 MuJoCo 中可能使用凸包近似，产生错误碰撞。仍保存吗？'):
                return
            if overwrite:
                if not messagebox.askyesno('修改原文件',
                        '将修改 scene_color.xml，先生成带时间戳的备份。是否继续？'):
                    return
                stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                backup=HERE/f'scene_color.xml.{stamp}.bak'
                shutil.copy2(SCENE,backup)
                SCENE.write_bytes(content.encode('utf-8'))
                status.set(f'已更新 scene_color.xml；备份 {backup.name}')
            else:
                EXPORT.write_bytes(content.encode('utf-8'))
                status.set(f'已生成 {EXPORT.name}；原场景文件未改动。')
            messagebox.showinfo('保存成功', status.get()+'\n重新加载 XML 才会应用到仿真。')
        except Exception as exc:
            messagebox.showerror('保存失败',str(exc))

    mode.trace_add('write',recolor)
    tree.bind('<<TreeviewSelect>>',on_selection)
    entry.bind('<Return>',lambda _event:save_notes())
    ttk.Button(right,text='保存当前标注及选择',command=save_notes).pack(fill='x',pady=(12,5))
    ttk.Button(right,text='镜头聚焦所选网格',command=focus).pack(fill='x',pady=3)

    toolbar=ttk.Frame(root)
    toolbar.grid(row=2,column=0,sticky='ew',padx=12,pady=(10,3))
    ttk.Button(toolbar,text='← 上一个',command=lambda:move(-1)).pack(side='left',padx=(0,8))
    ttk.Button(toolbar,text='下一个 →',command=lambda:move(1)).pack(side='left',padx=(0,16))
    ttk.Button(toolbar,text='导出新 XML（不修改原版）',command=lambda:save_xml(False)).pack(side='left',padx=(0,8))
    ttk.Button(toolbar,text='应用到原 XML（自动备份）',command=lambda:save_xml(True)).pack(side='left')
    ttk.Label(root,textvariable=status,wraplength=970).grid(row=3,column=0,sticky='w',padx=12,pady=(4,12))

    closing=False
    def close():
        nonlocal closing
        if closing: return
        closing=True
        try:
            save_notes()
            if viewer.is_running(): viewer.close()
        finally:
            root.destroy()
    def poll():
        if closing: return
        if not viewer.is_running():
            close();return
        viewer.sync()  # 不调用 mj_step：仅用于静态识别
        root.after(80,poll)
    root.protocol('WM_DELETE_WINDOW',close)
    if items:
        tree.selection_set(items[0]['name'])
        tree.focus(items[0]['name'])
        on_selection()
    root.after(80,poll)
    root.mainloop()


if __name__ == '__main__':
    main()
