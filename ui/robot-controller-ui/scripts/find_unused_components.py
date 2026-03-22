import os
import re

SRC_DIR = './src/components'   # Change to your component folder if needed

def all_tsx_files(root):
    files = []
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if f.endswith('.tsx') or f.endswith('.ts'):
                files.append(os.path.relpath(os.path.join(dirpath, f), '.'))
    return files

def get_component_name(filename):
    # Remove path and extension
    base = os.path.splitext(os.path.basename(filename))[0]
    return base

def scan_for_imports(src_dir, component_files):
    imported = set()
    pattern = re.compile(r'import\s+(?:.*\s+from\s+)?[\'"](.+)[\'"]')
    for dirpath, _, filenames in os.walk(src_dir):
        for fname in filenames:
            if fname.endswith('.ts') or fname.endswith('.tsx'):
                with open(os.path.join(dirpath, fname), encoding="utf-8") as f:
                    for line in f:
                        m = pattern.search(line)
                        if m:
                            # Try to resolve import path to a component filename
                            path = m.group(1)
                            for comp in component_files:
                                # Normalize path slashes for comparison
                                comp_no_ext = os.path.splitext(comp.replace('\\', '/'))[0]
                                if path in comp_no_ext or path.endswith('/' + get_component_name(comp)):
                                    imported.add(comp)
    return imported

def main():
    component_files = all_tsx_files(SRC_DIR)
    imported = scan_for_imports('./src', component_files)
    unused = set(component_files) - imported

    print("=== POSSIBLY UNUSED COMPONENTS ===")
    for file in sorted(unused):
        print(file)

if __name__ == '__main__':
    main()
