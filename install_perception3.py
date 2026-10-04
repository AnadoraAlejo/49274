#!/usr/bin/env python3
"""Run with the package directory containing setup.py (not workspace root)."""
import ast
from pathlib import Path
import shutil
import sys


def patch(source):
    if 'from cave_explorer.perception3_localisation import Perception3' in source:
        raise ValueError('Perception 3 is already installed')
    required = ['class CaveExplorer(Node):', 'self.localise_artifact()',
                'detections = stop_sign_model.detectMultiScale',
                'self.main_loop_timer_ = self.create_timer(0.2, self.main_loop)']
    if not all(text in source for text in required):
        raise ValueError('File differs from supplied starter/P1 version. Use README manual integration.')
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CaveExplorer')
    lines = source.splitlines(keepends=True)
    replacements = {
        'localise_artifact': '''    def localise_artifact(self, image_msg, detections):
        # detections: (label, (x, y, width, height)) in original RGB pixels
        self.perception3.submit(image_msg, detections)

''',
        'publish_artifact_markers': '''    def publish_artifact_markers(self):
        self.perception3.publish_markers()

'''
    }
    for node in sorted(cls.body, key=lambda n: n.lineno, reverse=True):
        if isinstance(node, ast.FunctionDef) and node.name in replacements:
            lines[node.lineno-1:node.end_lineno] = [replacements[node.name]]
    result = ''.join(lines)
    result = result.replace('import math\n', 'import math\nfrom cave_explorer.perception3_localisation import Perception3\n', 1)
    result = result.replace('        self.main_loop_timer_ =', '        self.perception3 = Perception3(self)\n\n        self.main_loop_timer_ =', 1)
    result = result.replace('self.localise_artifact()',
                            "self.localise_artifact(image_msg, [('stop_sign', tuple(box)) for box in detections])", 1)
    result = result.replace('(x + height, y + width)', '(x + width, y + height)')
    result = result.replace('        self.image_detections_pub_.publish(image_detection_message)',
                            '        image_detection_message.header = image_msg.header\n        self.image_detections_pub_.publish(image_detection_message)', 1)
    ast.parse(result)
    return result


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python3 install_perception3.py /path/to/cave_explorer')
    package = Path(sys.argv[1]).expanduser().resolve()
    target = package/'cave_explorer'/'cave_explorer.py'
    if not (package/'setup.py').is_file() or not target.is_file():
        raise SystemExit('Choose the package folder containing setup.py and cave_explorer/cave_explorer.py')
    source = target.read_text()
    result = patch(source)
    backup = target.with_suffix('.py.before_perception3')
    if backup.exists():
        raise SystemExit('Backup exists; refusing to overwrite it')
    modules = ['perception3_geometry.py', 'perception3_localisation.py']
    if any((target.parent/name).exists() for name in modules):
        raise SystemExit('Perception 3 module exists; refusing to overwrite it')
    shutil.copy2(target, backup)
    for name in modules:
        shutil.copy2(Path(__file__).parent/name, target.parent/name)
    target.write_text(result)
    print('Installed. Original backed up at:', backup)
    print('Perception 1 methods preserved. Rebuild cave_explorer, then launch normally.')


if __name__ == '__main__':
    main()
