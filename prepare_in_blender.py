# -*- coding: utf-8 -*-
"""
Blender 4.x: 运行本脚本，不要用 PyCharm 的普通 Python 解释器运行。
作用：从未被 1.py 修改过的 2027rmuc.obj 导入场地，减面后导出轻量 OBJ，
      如原模型包含可读取的不同材质颜色，则自动拆分并给 MuJoCo 设置颜色。
      同时生成 scene_fast.xml 供 view.py 使用。

使用：在 Blender -> Scripting -> Open 打开本脚本 -> Run Script。
      建议关闭其他大模型工程，以免叠加占用内存。
"""
from pathlib import Path
from collections import Counter
import re
import sys
import xml.etree.ElementTree as ET

try:
    import bpy
except ImportError as e:
    raise RuntimeError("请在 Blender 的 Scripting 页面运行本文件，而不是 PyCharm！") from e

# Blender Text Editor 使用 Run Script 时，__file__ 可能不是外部脚本真实路径。
# 优先读取当前打开的 Text 数据块的 filepath；命令行模式才用 __file__。
text_block = getattr(bpy.context, 'edit_text', None)
if text_block is not None and text_block.filepath:
    script_path = Path(bpy.path.abspath(text_block.filepath)).resolve()
    if not script_path.is_file():
        script_path = Path(__file__).resolve()
else:
    script_path = Path(__file__).resolve()
ROOT = script_path.parent
SOURCE = ROOT / "meshes" / "2027rmuc.obj"
OUT_DIR = ROOT / "meshes" / "optimized"
SCENE_XML = ROOT / "scene_fast.xml"

# 保留原面数的百分比。0.05 = 尽量只保留约 5% 的面；可改成 0.02 或 0.10。
DECIMATE_RATIO = 0.05
# 最多导出多少个独立颜色的网格，以免大量 draw calls 再次引起卡顿。
MAX_PARTS = 32
DEFAULT_COLOR = (0.27, 0.29, 0.33)

if not SOURCE.is_file():
    raise FileNotFoundError(f"没有找到原始 OBJ: {SOURCE}")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def material_color(mat):
    """取可读取的基础颜色；无法读取真实贴图颜色时不能凭空还原。"""
    if mat is None:
        return DEFAULT_COLOR
    name = mat.name.lower()
    # 仅针对名称中有清晰红蓝标记的材质做保守的颜色提示，不是还原真实贴图。
    if re.search(r"(^|[_. -])(red|hong)([_. -]|$)", name) or "红" in name:
        return (0.67, 0.12, 0.15)
    if re.search(r"(^|[_. -])(blue|lan)([_. -]|$)", name) or "蓝" in name:
        return (0.11, 0.23, 0.66)

    rgb = tuple(float(c) for c in mat.diffuse_color[:3])
    if mat.use_nodes and mat.node_tree:
        for node in mat.node_tree.nodes:
            if node.type == 'BSDF_PRINCIPLED':
                slot = node.inputs.get('Base Color')
                if slot and not slot.is_linked:
                    rgb = tuple(float(v) for v in slot.default_value[:3])
                break
    return tuple(min(1.0, max(0.0, v)) for v in rgb)


def colors_used(ob):
    used_indices = {p.material_index for p in ob.data.polygons}
    values = []
    for i in used_indices:
        mat = ob.data.materials[i] if i < len(ob.data.materials) else None
        values.append(tuple(round(x, 2) for x in material_color(mat)))
    return values


def update_scene(parts):
    model = ET.Element('mujoco', {'model': 'RMUC2027_field_optimized'})
    ET.SubElement(model, 'compiler', {'angle': 'degree', 'meshdir': 'meshes/optimized'})
    ET.SubElement(model, 'statistic', {'center': '14 7.5 1', 'extent': '22'})
    ET.SubElement(model, 'option', {'timestep': '0.005', 'gravity': '0 0 -9.81'})
    assets = ET.SubElement(model, 'asset')
    world = ET.SubElement(model, 'worldbody')
    ET.SubElement(world, 'light', {
        'name': 'sun', 'directional': 'true', 'pos': '14 7.5 18',
        'dir': '0 0 -1', 'diffuse': '0.85 0.85 0.85'
    })
    ET.SubElement(world, 'camera', {
        'name': 'overview', 'pos': '14 -20 18', 'xyaxes': '1 0 0 0 0.53 0.85'
    })
    for idx, (filename, color) in enumerate(parts):
        mesh_name = f'field_part_{idx:02d}'
        ET.SubElement(assets, 'mesh', {'name': mesh_name, 'file': filename})
        ET.SubElement(world, 'geom', {
            'name': mesh_name, 'type': 'mesh', 'mesh': mesh_name,
            'contype': '0', 'conaffinity': '0',
            'rgba': f'{color[0]:.4f} {color[1]:.4f} {color[2]:.4f} 1'
        })
    # 目前仅用于简单的地面碰撞；不是与场地模型吻合的碰撞网格。
    ET.SubElement(world, 'geom', {
        'name': 'temporary_floor', 'type': 'plane',
        'pos': '14 7.5 -0.30', 'size': '14 8 0.1',
        'rgba': '0.10 0.10 0.12 1'
    })
    ET.indent(model, space='  ')
    SCENE_XML.write_text(ET.tostring(model, encoding='unicode'), encoding='utf-8')


print(f'[1/5] 读取原始 OBJ: {SOURCE} ({SOURCE.stat().st_size / 1024**2:.1f} MiB)')
# 先清空 Blender 默认场景；本脚本专用于空的 Blender 会话。
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
# 用户原 OBJ 在 MuJoCo 中 Z 轴朝上，因此两次均用 Z up，保持坐标一致。
bpy.ops.wm.obj_import(
    filepath=str(SOURCE), forward_axis='Y', up_axis='Z',
    use_split_objects=False, use_split_groups=False
)
objects = [o for o in bpy.context.scene.objects if o.type == 'MESH' and len(o.data.polygons)]
if not objects:
    raise RuntimeError('没有读出任何网格对象；请核对 OBJ 文件。')
print(f'[2/5] 导入 {len(objects)} 个网格，开始减面...')
for obj in objects:
    before = len(obj.data.polygons)
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    if before >= 100:
        modifier = obj.modifiers.new('MuJoCo_LowPoly', 'DECIMATE')
        modifier.ratio = DECIMATE_RATIO
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    after = len(obj.data.polygons)
    print(f'  {obj.name}: {before:,} -> {after:,} faces')

print('[3/5] 检查可读取的材质颜色...')
all_colors = {tuple(c) for o in objects for c in colors_used(o)}
# 默认纯白/灰的多个虚拟材质不是有效的颜色来源。
meaningful_colors = len(all_colors) >= 2
material_count = sum(len(o.data.materials) for o in objects)
if material_count and not meaningful_colors:
    print('  原模型存在材质名称，但无法读出不同基础颜色；将输出灰色单一网格。')
if not material_count:
    print('  原模型无材质数据；将输出灰色单一网格。')

# 若读取到了不同的基础颜色，可拆分为多个网格，并以各自的颜色显示。
if meaningful_colors and sum(len(o.data.materials) for o in objects) <= MAX_PARTS:
    print('  检测到多种颜色，按材质拆分以生成多色 MuJoCo 模型...')
    for obj in list(objects):
        if len(set(p.material_index for p in obj.data.polygons)) <= 1:
            continue
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.separate(type='MATERIAL')
        bpy.ops.object.mode_set(mode='OBJECT')
    objects = [o for o in bpy.context.scene.objects if o.type == 'MESH' and len(o.data.polygons)]
    if len(objects) > MAX_PARTS:
        print(f'  警告：拆分出 {len(objects)} 个物体，可能影响绘制速度。')
else:
    if meaningful_colors:
        print('  材质组过多，暂不拆分；优先保证流畅性。')

print(f'[4/5] 输出 {len(objects)} 个轻量化 OBJ...')
parts = []
for i, obj in enumerate(objects):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    filename = f'part_{i:02d}.obj'
    path = OUT_DIR / filename
    bpy.ops.wm.obj_export(
        filepath=str(path), forward_axis='Y', up_axis='Z',
        export_selected_objects=True,
        apply_modifiers=True,
        export_uv=False, export_normals=True, export_materials=False,
        export_triangulated_mesh=True
    )
    if meaningful_colors:
        used = {p.material_index for p in obj.data.polygons}
        idx = next(iter(used), 0)
        mat = obj.data.materials[idx] if idx < len(obj.data.materials) else None
        rgba = material_color(mat)
    else:
        rgba = DEFAULT_COLOR
    parts.append((filename, rgba))
    print(f'  {filename}  {path.stat().st_size / 1024**2:.1f} MiB  颜色={rgba}')

print('[5/5] 生成 scene_fast.xml')
update_scene(parts)
print('成功！可回到 PyCharm 运行 view.py。')
print('提示：颜色只来自可读取的原始基础材质，纹理图片并不会自动恢复。')
