#!/usr/bin/env python3
# fix_sysroot_symlinks.py

import os
from pathlib import Path

SYSROOT = Path("/home/poledna/nvidia_ws/target_fs")

def fix_symlink(symlink_path):
    """Fix a symlink to be relative within sysroot"""
    
    # Read current target
    current_target = os.readlink(symlink_path)
    
    # Only process absolute symlinks (like /lib/foo.so.1)
    if not os.path.isabs(current_target):
        return False, "already relative"
    
    # The absolute target should exist within SYSROOT
    # e.g., /lib/aarch64-linux-gnu/librt.so.1 
    #    -> /home/poledna/nvidia_ws/target_fs/lib/aarch64-linux-gnu/librt.so.1
    target_in_sysroot = SYSROOT / current_target.lstrip('/')
    
    if not target_in_sysroot.exists():
        return False, f"target doesn't exist: {current_target}"
    
    # Calculate relative path from symlink's directory to the target
    # Both paths are inside SYSROOT
    try:
        relative_path = os.path.relpath(target_in_sysroot, symlink_path.parent)
        
        # Remove old symlink and create new one
        symlink_path.unlink()
        symlink_path.symlink_to(relative_path)
        
        return True, f"{current_target} -> {relative_path}"
        
    except (ValueError, OSError) as e:
        return False, f"error: {e}"

def main():
    print(f"🔧 Fixing absolute symlinks in: {SYSROOT}")
    print(f"Converting /lib/... paths to relative paths within sysroot\n")
    
    fixed = 0
    skipped = 0
    failed = 0
    
    # Find all symlinks
    for symlink in SYSROOT.rglob('*'):
        if not symlink.is_symlink():
            continue
        
        success, message = fix_symlink(symlink)
        
        if success:
            fixed += 1
            # Show first 20 fixes as examples
            if fixed <= 20:
                rel = symlink.relative_to(SYSROOT)
                print(f"✅ {rel}")
                print(f"   {message}\n")
        elif "already relative" in message:
            skipped += 1
        else:
            failed += 1
            if failed <= 10:
                rel = symlink.relative_to(SYSROOT)
                print(f"⚠️  {rel}: {message}")
    
    print(f"\n{'='*60}")
    print(f"📊 Summary:")
    print(f"  ✅ Fixed: {fixed} symlinks")
    print(f"  ⏭️  Skipped (already relative): {skipped}")
    print(f"  ❌ Failed: {failed}")
    print(f"{'='*60}")
    

if __name__ == '__main__':
    main()
