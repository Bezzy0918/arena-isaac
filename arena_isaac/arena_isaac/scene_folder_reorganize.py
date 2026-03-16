#!/usr/bin/env python3
"""
Interactive Hospital Asset Reorganizer

This script will:
1. Create an "Object" folder
2. Move Materials and Props into Object
3. For each .usd file in Props:
   - Create a folder named after the file (without .usd extension)
   - Move the .usd file into that folder
4. Move all prop folders to be parallel with Materials
5. Remove empty Props folder

Final structure:
Hospital/
├── hospital.usd
└── Object/
    ├── Materials/
    ├── prop1/
    │   └── prop1.usd
    ├── prop2/
    │   └── prop2.usd
    └── ...
"""

import os
import shutil
from pathlib import Path
import sys

def get_hospital_path():
    """Get the Hospital directory path from user"""
    print("=" * 70)
    print("Hospital Asset Reorganizer")
    print("=" * 70)
    
    default_path = "~/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments/Hospital"
    
    print(f"\nDefault path: {default_path}")
    user_input = input("Enter Hospital directory path (or press Enter for default): ").strip()
    
    if not user_input:
        user_input = default_path
    
    path = Path(user_input).expanduser().resolve()
    
    return path


def verify_directory(path):
    """Verify the directory structure"""
    if not path.exists():
        print(f"\n❌ Error: Directory does not exist: {path}")
        return False
    
    print(f"\n✓ Found directory: {path}")
    print("\n📂 Current contents:")
    
    items = list(path.iterdir())
    for item in sorted(items):
        icon = "📁" if item.is_dir() else "📄"
        print(f"  {icon} {item.name}")
        
        # Show Props contents
        if item.name == "Props" and item.is_dir():
            usd_files = list(item.glob("*.usd"))
            print(f"      ({len(usd_files)} USD files)")
    
    return True


def confirm_action():
    """Ask user to confirm"""
    print("\n" + "=" * 70)
    print("⚠️  This will reorganize the folder structure!")
    print("=" * 70)
    
    response = input("\nProceed with reorganization? (yes/no): ").strip().lower()
    return response in ['yes', 'y']


def reorganize(base_path):
    """Perform the reorganization"""
    
    print("\n" + "=" * 70)
    print("Starting reorganization...")
    print("=" * 70)
    
    # Step 1: Create Object folder
    print("\n[1/6] Creating Object folder...")
    object_dir = base_path / "Object"
    object_dir.mkdir(exist_ok=True)
    print(f"   ✓ Created: Object/")
    
    # Step 2: Move Materials
    print("\n[2/6] Moving Materials...")
    materials_src = base_path / "Materials"
    if materials_src.exists():
        materials_dst = object_dir / "Materials"
        if not materials_dst.exists():
            shutil.move(str(materials_src), str(materials_dst))
            print(f"   ✓ Materials/ → Object/Materials/")
        else:
            print(f"   ⚠ Object/Materials/ already exists")
    else:
        print(f"   ⚠ Materials/ not found")
    
    # Step 3: Move Props
    print("\n[3/6] Moving Props...")
    props_src = base_path / "Props"
    if props_src.exists():
        props_dst = object_dir / "Props"
        if not props_dst.exists():
            shutil.move(str(props_src), str(props_dst))
            print(f"   ✓ Props/ → Object/Props/")
        else:
            print(f"   ⚠ Object/Props/ already exists")
    else:
        print(f"   ⚠ Props/ not found")
        return False
    
    # Step 4: Process USD files
    print("\n[4/6] Creating folders for USD files...")
    props_dir = object_dir / "Props"
    
    if not props_dir.exists():
        print(f"   ❌ Props directory not found")
        return False
    
    usd_files = list(props_dir.glob("*.usd"))
    print(f"   Found {len(usd_files)} USD files")
    
    for i, usd_file in enumerate(usd_files, 1):
        folder_name = usd_file.stem
        prop_folder = props_dir / folder_name
        prop_folder.mkdir(exist_ok=True)
        
        dst_file = prop_folder / usd_file.name
        shutil.move(str(usd_file), str(dst_file))
        
        print(f"   [{i}/{len(usd_files)}] {folder_name}/ ← {usd_file.name}")
    
    # Step 5: Move prop folders to Object
    print("\n[5/6] Moving prop folders to Object/...")
    prop_folders = [d for d in props_dir.iterdir() if d.is_dir()]
    
    for i, prop_folder in enumerate(prop_folders, 1):
        dst_folder = object_dir / prop_folder.name
        
        if not dst_folder.exists():
            shutil.move(str(prop_folder), str(dst_folder))
            print(f"   [{i}/{len(prop_folders)}] {prop_folder.name}/ → Object/")
        else:
            print(f"   [{i}/{len(prop_folders)}] ⚠ {prop_folder.name}/ already exists")
    
    # Step 6: Remove empty Props
    print("\n[6/6] Cleaning up...")
    try:
        if not any(props_dir.iterdir()):
            props_dir.rmdir()
            print(f"   ✓ Removed empty Props/ folder")
        else:
            print(f"   ⚠ Props/ not empty, keeping it")
    except Exception as e:
        print(f"   ⚠ Could not remove Props/: {e}")
    
    return True


def show_final_structure(base_path):
    """Show the final structure"""
    print("\n" + "=" * 70)
    print("✅ Reorganization Complete!")
    print("=" * 70)
    
    print(f"\n📋 Final structure:")
    
    object_dir = base_path / "Object"
    
    print(f"\n{base_path.name}/")
    
    # Show files in root
    for item in sorted(base_path.iterdir()):
        if item.is_file():
            print(f"├── {item.name}")
    
    # Show Object folder
    if object_dir.exists():
        print(f"└── Object/")
        
        items = sorted(object_dir.iterdir(), key=lambda x: (not x.is_dir(), x.name))
        for i, item in enumerate(items[:20]):  # Show first 20
            is_last = i == len(items) - 1 and len(items) <= 20
            prefix = "└──" if is_last else "├──"
            print(f"    {prefix} {item.name}/")
            
            # Show one file inside each prop folder
            if item.is_dir() and item.name != "Materials":
                sub_files = list(item.glob("*.usd"))
                if sub_files:
                    sub_prefix = "    " if is_last else "│   "
                    print(f"{sub_prefix}    └── {sub_files[0].name}")
        
        if len(items) > 20:
            print(f"    └── ... ({len(items) - 20} more folders)")


def main():
    """Main function"""
    try:
        # Get path
        hospital_path = get_hospital_path()
        
        # Verify
        if not verify_directory(hospital_path):
            return 1
        
        # Confirm
        if not confirm_action():
            print("\n❌ Operation cancelled by user")
            return 0
        
        # Reorganize
        success = reorganize(hospital_path)
        
        if success:
            # Show result
            show_final_structure(hospital_path)
            print("\n✅ All done!\n")
            return 0
        else:
            print("\n❌ Reorganization failed")
            return 1
            
    except KeyboardInterrupt:
        print("\n\n❌ Operation cancelled by user")
        return 1
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())