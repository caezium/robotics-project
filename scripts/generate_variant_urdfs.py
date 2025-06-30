import os
import xml.etree.ElementTree as ET
import random

URDF_DIR = '../assets/urdf/ycb'
VARIANT_DIR = '../assets/urdf/ycb_variants'
os.makedirs(VARIANT_DIR, exist_ok=True)

def random_color():
    return [random.random() for _ in range(3)] + [1.0]

for urdf_file in os.listdir(URDF_DIR):
    if urdf_file.endswith('.urdf'):
        tree = ET.parse(os.path.join(URDF_DIR, urdf_file))
        root = tree.getroot()
        for i in range(5):  # 5 variants per object
            # CHANGE COLOR
            for mat in root.iter('material'):
                color = random_color()
                for el in mat:
                    if el.tag == 'color':
                        el.set('rgba', ' '.join(map(str, color)))
            # CHANGE SCALE and UPDATE MESH PATH
            for mesh in root.iter('mesh'):
                # Get the original scale (default to 1 1 1 if not present)
                orig_scale = mesh.get('scale')
                if orig_scale:
                    orig_scale = [float(x) for x in orig_scale.split()]
                else:
                    orig_scale = [1.0, 1.0, 1.0]
                # RANDOME SCALE CHANGE MULT HERE
                scale = [str(orig_scale[i] * random.uniform(0.95, 1.05)) for i in range(3)]
                mesh.set('scale', ' '.join(scale))
                fname = mesh.get('filename')
                if fname and not fname.startswith('ycb/'):
                    mesh.set('filename', f'ycb/{fname}')
            # Save new URDF
            new_name = urdf_file.replace('.urdf', f'_var{i}.urdf')
            tree.write(os.path.join(VARIANT_DIR, new_name))