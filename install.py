#!/usr/bin/env python3
"""Install additive modules into the attached package; preserve starter code."""
import argparse
import ast
import re
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('package',help='Folder containing setup.py, package.xml and cave_explorer/')
    args=parser.parse_args()
    package=Path(args.package).expanduser().resolve()
    here=Path(__file__).resolve().parent
    setup=package/'setup.py'; xml=package/'package.xml'
    if not setup.is_file() or not xml.is_file() or not (package/'cave_explorer/cave_explorer.py').is_file():
        raise SystemExit('Choose the cave_explorer PACKAGE folder, not the workspace or src folder')
    source=setup.read_text()
    if 'cave_mission =' in source:
        raise SystemExit('Already installed. Keep this version or restore backups before installing again.')
    pattern=r"(['\"])cave_explorer\s*=\s*cave_explorer\.cave_explorer:main\1"
    match=re.search(pattern,source)
    if match is None:
        raise SystemExit('Unrecognised setup.py: see README manual entry-point instructions')
    # Insert after the known string while preserving valid trailing commas.
    new=source[:match.end()]+",\n            'cave_mission = cave_explorer.mission_node:main'"+source[match.end():]
    ast.parse(new)
    tree=ET.fromstring(xml.read_text())
    existing={e.text for e in tree if e.tag in ('depend','exec_depend')}
    deps=['rclpy','sensor_msgs','std_msgs','geometry_msgs','nav_msgs','nav2_msgs','action_msgs',
          'visualization_msgs','cv_bridge','tf2_ros','ament_index_python','python3-numpy','python3-opencv']
    addition=''.join(f'  <exec_depend>{name}</exec_depend>\n' for name in deps if name not in existing)
    new_xml=xml.read_text().replace('</package>',addition+'</package>')
    ET.fromstring(new_xml)
    writes=[]
    for folder in ('cave_explorer','launch'):
        for src in sorted((here/folder).glob('*.py')):
            target=package/folder/src.name
            if target.exists() and target.read_bytes()!=src.read_bytes():
                raise SystemExit('Existing different file: '+str(target)+'; use a fresh copy of the attached package')
            writes.append((src,target))
    for path in (setup,xml):
        backup=path.with_name(path.name+'.before_p3_a1')
        if backup.exists():
            raise SystemExit('Backup already exists: '+str(backup))
    # All validation precedes writes.
    for path in (setup,xml):
        shutil.copy2(path,path.with_name(path.name+'.before_p3_a1'))
    for src,target in writes:
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(src,target)
    setup.write_text(new); xml.write_text(new_xml)
    print('Installed into',package)
    print('Original cave_explorer.py unchanged. Rebuild, then use planning3_advanced1.launch.py for autonomy.')


if __name__=='__main__':
    main()
