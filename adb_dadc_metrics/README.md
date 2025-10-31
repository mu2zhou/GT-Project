
# ADB DADC Metrics — Final Fixed Build

Run either as a **script** or **module** without import issues.
- Script mode uses **package import** (adds project parent to `sys.path`).
- `pipeline.py` writes scalars via `"
".join(...)` (no f-string pitfalls).
