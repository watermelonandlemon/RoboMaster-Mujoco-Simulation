# 场地文件
与v0.3相比,将v0.3的附件2027rmuc.obj导入Blender4.5,并用Blender4.5运行脚本,得到减面后的场地模型part_00.obj;

Blender4.5运行的脚本内容是prepare_in_blender.py;

若不经过减面处理,2027rmuc.obj用Mujoco打开后会很卡;

运行view.py即可通过Mujoco显示场地;

场地为灰白,不包含颜色信息;
