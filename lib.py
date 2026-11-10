import math

# 生成绕指定轴旋转的四元数
def axis_quat(axis, angle):
    half = math.radians(angle) / 2
    c = math.cos(half)
    s = math.sin(half)

    if axis == "x":
        return (c, s, 0, 0)
    if axis == "y":
        return (c, 0, s, 0)
    return (c, 0, 0, s)

# 四元数相乘
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )



#print("MuJoCo version:", mujoco.__version__)
#print("Loading optimized scene:", SCENE)
#print(f"Loaded: {model.nmesh} meshes, {model.ngeom} geoms")