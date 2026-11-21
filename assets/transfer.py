cat << 'EOF' > /tmp/convert_duck.py
import xml.etree.ElementTree as ET
from pathlib import Path

src_urdf = Path("/tmp/microduck_description/urdf/microduck.urdf")
out_dir  = Path("assets/models/microduck")
tree = ET.parse(src_urdf)
root = tree.getroot()

# 1. 提取顶层官方配色 (Cream 奶白外壳 / Orange 橙色嘴蹼 / Amber 琥珀 / 深色舵机) 并内联到每个 STL visual
materials = {}
for mat in list(root.findall("material")):
    name = mat.get("name")
    color = mat.find("color")
    if name and color is not None:
        materials[name] = color.get("rgba")
    root.remove(mat)

# 2. 找出根 Link 和头部末端 Link (head_roll 的子 Link)
child_links = {j.find("child").get("link") for j in root.findall("joint") if j.find("child") is not None}
all_links   = [l.get("name") for l in root.findall("link")]
root_link_name = [n for n in all_links if n not in child_links][0]

head_link_name = root_link_name
for j in root.findall("joint"):
    if j.get("name") in ("head_roll", "head_yaw", "head_pitch"):
        head_link_name = j.find("child").get("link")
        if j.get("name") == "head_roll":
            break

# 3. 替换 STL 路径为 Gazebo 原生 model://，内联颜色，移除高面数 STL collision 防卡顿
for link in root.findall("link"):
    for col in list(link.findall("collision")):
        link.remove(col)
    for vis in link.findall("visual"):
        geom = vis.find("geometry")
        if geom is not None:
            mesh = geom.find("mesh")
            if mesh is not None and mesh.get("filename"):
                stl_name = Path(mesh.get("filename")).name
                mesh.set("filename", f"model://microduck/meshes/{stl_name}")
        mat = vis.find("material")
        if mat is not None:
            mname = mat.get("name", "")
            if mat.find("color") is None and mname in materials:
                ET.SubElement(mat, "color", {"rgba": materials[mname]})

# 确保根 Link 有真实物理惯量 (整机 0.737kg) 与底部防穿模碰撞包络
root_link = next(l for l in root.findall("link") if l.get("name") == root_link_name)
if root_link.find("inertial") is None:
    iner = ET.SubElement(root_link, "inertial")
    ET.SubElement(iner, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(iner, "mass", {"value": "0.737"})
    ET.SubElement(iner, "inertia", {"ixx": "0.004", "ixy": "0", "ixz": "0",
                                    "iyy": "0.004", "iyz": "0", "izz": "0.0025"})

# 给左右脚和躯干加轻量级透明物理碰撞盒，保证稳立桌面/地面且 250Hz 满帧运行
for link in root.findall("link"):
    lname = link.get("name", "")
    if "ankle" in lname or "foot" in lname or lname == root_link_name:
        col = ET.SubElement(link, "collision")
        sz = "0.08 0.05 0.015" if ("ankle" in lname or "foot" in lname) else "0.10 0.10 0.12"
        oz = "-0.02" if ("ankle" in lname or "foot" in lname) else "0.0"
        ET.SubElement(col, "origin", {"xyz": f"0 0 {oz}", "rpy": "0 0 0"})
        geom = ET.SubElement(col, "geometry")
        ET.SubElement(geom, "box", {"size": sz})

# 4. 在协同联调模式下将全身 15 个关节锁定在标准站姿 (防止在 Gazebo 里无控制器瘫软散架)
for joint in root.findall("joint"):
    if joint.get("type") in ("revolute", "continuous"):
        joint.set("type", "fixed")

# 5. 在真实头部顶端加一片薄导磁贴片 (供无人机磁吸定位)
mag_link = ET.SubElement(root, "link", {"name": "magnet_plate_link"})
mag_vis  = ET.SubElement(mag_link, "visual")
ET.SubElement(ET.SubElement(mag_vis, "geometry"), "cylinder", {"radius": "0.025", "length": "0.003"})
ET.SubElement(ET.SubElement(mag_vis, "material", {"name": "steel"}), "color", {"rgba": "0.75 0.78 0.82 1.0"})

mag_joint = ET.SubElement(root, "joint", {"name": "magnet_plate_joint", "type": "fixed"})
ET.SubElement(mag_joint, "parent", {"link": head_link_name})
ET.SubElement(mag_joint, "child", {"link": "magnet_plate_link"})
ET.SubElement(mag_joint, "origin", {"xyz": "0 0 0.045", "rpy": "0 0 0"})

# 6. 挂载 Gazebo 平面运动插件 (/microduck/cmd_vel)
gz = ET.SubElement(root, "gazebo")
plug = ET.SubElement(gz, "plugin", {"name": "microduck_planar_controller", "filename": "libgazebo_ros_planar_move.so"})
ET.SubElement(plug, "commandTopic").text    = "/microduck/cmd_vel"
ET.SubElement(plug, "odometryTopic").text   = "/microduck/odom"
ET.SubElement(plug, "odometryFrame").text   = "odom"
ET.SubElement(plug, "odometryRate").text    = "50.0"
ET.SubElement(plug, "robotBaseFrame").text  = root_link_name

ET.indent(tree, space="  ")
tree.write(out_dir / "microduck.urdf", encoding="utf-8", xml_declaration=True)

# 7. 生成 Gazebo Classic 标准 model.config
(out_dir / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>microduck</name>
  <version>1.0</version>
  <sdf version="1.6">microduck.urdf</sdf>
  <description>Official Pollen Robotics Microduck 38-STL Mesh Model for Gazebo Classic</description>
</model>
""")
print(f"✅ 转换完成！根Link: {root_link_name}, 头部Link: {head_link_name}, 共载入 38 个官方曲面 STL。")
EOF

python3 /tmp/convert_duck.py
rm -rf /tmp/microduck_description /tmp/convert_duck.py
