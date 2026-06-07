import xml.etree.ElementTree as ET
try:
    tree = ET.parse('/home/robot/hospital_ws/src/hospital_world_bridge/worlds/hospital.world')
    root = tree.getroot()
    world = root.find('world')
    for model in world.findall('model'):
        pose = model.find('pose')
        if pose is not None:
            vals = [float(v) for v in pose.text.split()]
            x, y = vals[0], vals[1]
            if abs(x - (-3.5)) < 1.5 and abs(y - 1.0) < 1.5:
                name = model.get("name")
                print(name, pose.text)
except Exception as e:
    print(e)
