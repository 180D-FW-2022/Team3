# -*- mode: python ; coding: utf-8 -*-
<<<<<<< HEAD
import os

from kivy_deps import sdl2, glew

from kivymd import hooks_path as kivymd_hooks_path

path = os.path.abspath(".")

=======
from kivy_deps import sdl2, glew
from kivymd import hooks_path as kivymd_hooks_path

>>>>>>> 0b15c864751a04f17bbf729f1833392ad25ce1b6
block_cipher = None


a = Analysis(
<<<<<<< HEAD
    ['KitchenGUI.py'],
    pathex=[path],
    binaries=[],
    datas=[("../firebase_key.json", ".")],
=======
    ['kitchenGUI.py'],
    pathex=[],
    binaries=[],
    datas=[('firebase_key.json', '.')],
>>>>>>> 0b15c864751a04f17bbf729f1833392ad25ce1b6
    hiddenimports=[],
    hookspath=[kivymd_hooks_path],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
<<<<<<< HEAD

Key = ['numpy','PIL', 'cryptography', 'cv2', 'Pythonwin']

def remove_from_list(input, keys):
    outlist = []
    for item in input:
        name, _, _ = item
        flag = 0
        for key_word in keys:
            if name.find(key_word) > -1:
                flag = 1
        if flag != 1:
            outlist.append(item)
    return outlist

print("binaries")
for i in a.binaries:
    print(i)
a.binaries = remove_from_list(a.binaries, Key)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(pyz, Tree("C:\\Users\\Kenson\\Desktop\\GridApp\\"),
=======
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
>>>>>>> 0b15c864751a04f17bbf729f1833392ad25ce1b6
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
<<<<<<< HEAD
    *[Tree(p) for p in (sdl2.dep_bins + glew.dep_bins)],
    [],
    name='KitchenGUI',
=======
    [],
    name='kitchenGUI',
>>>>>>> 0b15c864751a04f17bbf729f1833392ad25ce1b6
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
<<<<<<< HEAD
)

coll = COLLECT(exe, Tree("C:\\Users\\Kenson\\Desktop\\GridApp\\"),
               a.binaries,
               a.zipfiles,
               a.datas,
               *[Tree(p) for p in (sdl2.dep_bins + glew.dep_bins)],
               strip=False,
               upx=True,
               name='touchtracer')
=======
    #...
    *[Tree(p) for p in (sdl2.dep_bins + glew.dep_bins)],
    #....
)
>>>>>>> 0b15c864751a04f17bbf729f1833392ad25ce1b6
