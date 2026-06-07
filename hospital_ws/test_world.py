import xml.etree.ElementTree as ET
try:
    tree = ET.parse('/opt/ros/jazzy/share/aws_robomaker_hospital_world/worlds/hospital.world')
    root = tree.getroot()
    world = root.find('world')
    for model in world.findall('model'):
        pose = model.find('pose')
        if pose is not None:
            vals = [float(v) for v in pose.text.split()]
            x, y = vals[0], vals[1]
            if abs(x) < 2.0 and abs(y) < 2.0:
                print(f'{model.get("name")}: {pose.text}')
except Exception as e:
    print(e)
