# SolidWorks 负责机械设计，Blender 负责三维模型处理，Unity 负责场景显示和交互仿真，MuJoCo 负责动力学与控制仿真，Fluent 负责专业气动分析

# RMUC_2027map_unity.rar【Github原版】
Github网址为： https://github.com/diaofenyuan/RMUC_2027map_unity 的完整项目文件;
根据作者说，这是使用DJI官方模型RMUC2027_V1.0.0.stp经OCCT三角化+Blender导出得到的;
可以直接导入为Unity或Blender;

# 2027rmuc.rar【Blender版】
里面是2027rmuc.blend;
是“RMUC_2027map_unity.rar”得Project\Assets\Field\field_visual_m.fbx被Import到Blender空白工程后得到的Blender工程文件;
打开即可得到2027地图;

# RMUC_2027_Unity_2022.rar【Unity版】
解压后的project文件夹用Unity Editor 2022.3.62f1c1打开;

# RMUC_2027_Unity_6.rar【Unity版】
解压后的project文件夹用Unity Editor 6000.0.85f2打开;

# 【Solidworks版】
官方下载的文件太大了，电脑内存不够打不开;

# ./Unity安装文件/UnityHubSetup-x64.exe
Unity Hub的安装文件，必须安装，一个Unity Hub中可以安装并使用不同版本的Unity Editor;

# ./Unity安装文件/UnitySetup64-2022.3.62f1.exe
Unity Editor 2022.3版本的安装文件，挂梯子也不太能下载，通过香港服务器下载得到;

# ./Unity安装文件/UnitySetup64-6000.0.85f2.exe
Unity Edotor 6.0版本的安装文件，大陆与港澳无法下载，挂梯子也不太行，通过日美服务器下载得到;

# ./unusable/RMUC2026.zip
项管-崔*雨给的，用Unity Editor6.0也打不开，文件很小疑似缺东西;

# ./RM社区/...
从RM社区中下载的官方场地模型文件，其中“RMUC2026_V2.0.0.stp”和“RMUC2027_V1.0.0.stp”因为电脑内存不够打不开，据说可以先转化成小文件一部分一部分打开;

# RMUC_2027_Mujoco.rar【Mujoco版】
里面是2027rmuc.obj，是“2027rmuc.blend”导出的obj文件;
导入到新blender空白工程后，场地为灰白色(不包含颜色信息);

# RMUC_2027_Mujoco_python.rar【Mujoco版】
完整的工程，运行python文件即显示地图;
问题:目前场地地图不包含颜色信息;

# RMUC_2027_Mujoco_python_1.rar【Mujoco版】
完整的工程，运行python文件即显示地图和西交开源的飞镖(飞镖为橙色，在发射处、闸门后);
问题:目前场地地图不包含颜色信息;

# RMUC_2027_Mujoco_python_2.rar【Mujoco版】
完整的工程，运行python文件即显示地图和西交开源的飞镖(飞镖为橙色，在发射处、闸门后);
已解决:场地地图包含颜色信息;
不足:地板是白色的;

# RMUC_2027_Mujoco_python_3.rar【Mujoco版】
完整的工程，运行python文件即显示地图和西交开源的飞镖(飞镖为橙色，在发射处、闸门后);
已解决:场地地图包含颜色信息;
不足:地板是白色的;
添加了飞镖镖头的视觉视角，让运行时的自由视角初始化为场地正中央;


# v0.1
第一个工程;
导入了来自西安交大在RM社区发布的2026年非制导镖模型;
通过Mujoco软件打开,未使用python;
因未使用python,无法进行人为控制,打开后表现为飞镖的自由落体;

# v0.2
在V0.1的基础上,加入了python控制程序;
构建了控制面板,可以人为设定飞镖的发射位置、发射角度与发射初速度;

# v0.3
第一版场地文件2027rmuc.obj,是“2027rmuc.blend”导出的obj文件;
场地反向导入回Blender后显示为灰白色,即该文件不包含颜色信息;

