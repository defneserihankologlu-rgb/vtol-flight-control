# 📤 Uploading to GitHub Guide

## Quick Answer

**Changes are NOT automatically uploaded.** You need to:
1. `git add` - Stage changes
2. `git commit` - Save changes
3. `git push` - Upload to GitHub

---

## Step-by-Step Setup

### 1. Initialize Git Repository (if not already done)

```bash
cd ~/ucus_kontrol_ws
git init
```

### 2. Create/Check .gitignore

Make sure you have a `.gitignore` file that excludes build artifacts:

```bash
# Check if .gitignore exists
cat .gitignore
```

If it doesn't exist or is incomplete, create one:

```bash
# ROS 2 workspace ignores
build/
install/
log/
*.pyc
__pycache__/
*.db3
*.bag
```

### 3. Create GitHub Repository

1. Go to https://github.com
2. Click "New repository"
3. Name it (e.g., `vtol-flight-control`)
4. **Don't** initialize with README (you already have files)
5. Click "Create repository"

### 4. Add Remote and Push

```bash
cd ~/ucus_kontrol_ws

# Add all files (respects .gitignore)
git add .

# Create first commit
git commit -m "Initial commit: VTOL flight control system"

# Add GitHub remote (replace YOUR_USERNAME and REPO_NAME)
git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git

# Push to GitHub
git branch -M main
git push -u origin main
```

---

## Making Changes and Uploading

### Workflow for Updates

**1. Make your changes** (edit files, add features, etc.)

**2. Check what changed:**
```bash
cd ~/ucus_kontrol_ws
git status
```

**3. Stage changes:**
```bash
# Stage specific files
git add src/ucus_kontrol/src/vtol_controller_node.cpp

# Or stage all changes
git add .
```

**4. Commit changes:**
```bash
git commit -m "Description of what you changed"
```

**5. Push to GitHub:**
```bash
git push
```

---

## Example: After Adding New Feature

```bash
cd ~/ucus_kontrol_ws

# 1. Check what changed
git status

# 2. Stage changes
git add src/ucus_sensorleri/src/fake_gps_node.cpp
git add src/ucus_sensorleri/CMakeLists.txt

# 3. Commit
git commit -m "Added fake GPS sensor node"

# 4. Push to GitHub
git push
```

---

## Important: What Gets Ignored

Your `.gitignore` should exclude:
- ✅ `build/` - Build artifacts (don't commit)
- ✅ `install/` - Installed files (don't commit)
- ✅ `log/` - Build logs (don't commit)
- ✅ `*.db3` - Rosbag database files
- ✅ `*.bag` - Rosbag files

**What SHOULD be committed:**
- ✅ `src/` - All source code
- ✅ `*.md` - Documentation files
- ✅ `*.yaml` - Parameter files
- ✅ `*.launch.py` - Launch files
- ✅ `.gitignore` - Git ignore rules

---

## Common Commands

### Check Status
```bash
git status                    # See what changed
git diff                     # See actual changes
git log                      # See commit history
```

### Undo Changes (Before Commit)
```bash
git checkout -- filename     # Discard changes to file
git reset HEAD filename      # Unstage file
```

### Update from GitHub (if working on multiple computers)
```bash
git pull                    # Download latest changes
```

---

## Best Practices

1. **Commit often** - Small, logical commits
2. **Good commit messages** - Describe what changed
3. **Don't commit build files** - Use .gitignore
4. **Push regularly** - Don't wait too long

---

## Example Commit Messages

Good:
- `"Added fake GPS, barometer, and magnetometer sensors"`
- `"Fixed EKF altitude drift issue"`
- `"Updated launch file to include all sensor nodes"`
- `"Added documentation for forward flight mode"`

Bad:
- `"update"`
- `"fix"`
- `"changes"`

---

## Troubleshooting

### "Repository not found"
- Check remote URL: `git remote -v`
- Verify GitHub repository exists
- Check authentication

### "Permission denied"
- Use SSH keys or personal access token
- Or use HTTPS with credentials

### "Nothing to commit"
- All changes are already committed
- Or no files were modified

### "Large files"
- GitHub has 100MB file limit
- Use Git LFS for large files
- Or exclude large files in .gitignore

---

## Quick Reference

```bash
# Initial setup
git init
git add .
git commit -m "Initial commit"
git remote add origin https://github.com/USERNAME/REPO.git
git push -u origin main

# Regular updates
git add .
git commit -m "Description"
git push
```

---

**Remember:** Changes are NOT automatic. You must commit and push manually!

