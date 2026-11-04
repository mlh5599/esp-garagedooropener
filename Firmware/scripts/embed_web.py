# PlatformIO pre-build script: compiles the files in web/ into the firmware as
# web_files.h, so the pages ship with every firmware (and OTA) update and
# uploading them never touches the settings stored on the filesystem.
import hashlib
import os

Import("env")  # noqa: F821 (provided by PlatformIO)

TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript",
}

project_dir = env["PROJECT_DIR"]  # noqa: F821
web_dir = os.path.join(project_dir, "web")
out_dir = os.path.join(env.subst("$BUILD_DIR"), "generated")  # noqa: F821
out_path = os.path.join(out_dir, "web_files.h")

files = {}
for name in sorted(os.listdir(web_dir)):
    if os.path.splitext(name)[1] in TYPES:
        with open(os.path.join(web_dir, name), "rb") as f:
            files[name] = f.read()

# Browsers cache style.css for a day; a content hash in its URL makes them
# fetch the new one after an update.
css_hash = hashlib.sha1(files.get("style.css", b"")).hexdigest()[:8]
for name in files:
    if name.endswith(".html"):
        files[name] = files[name].replace(b'href="/style.css"', b'href="/style.css?v=' + css_hash.encode() + b'"')

lines = [
    "// Generated from web/ by scripts/embed_web.py. Do not edit.",
    "#pragma once",
    "#include <Arduino.h>",
    "",
    "struct WebFile",
    "{",
    "  const char *path;",
    "  const char *type;",
    "  const char *data; // in flash (PROGMEM), null-terminated",
    "  size_t length;",
    "};",
    "",
]
entries = []
for i, (name, data) in enumerate(files.items()):
    body = ",".join(str(b) for b in data + b"\0")
    lines.append(f"static const char WEB_FILE_{i}[] PROGMEM = {{{body}}};")
    entries.append(f'  {{"/{name}", "{TYPES[os.path.splitext(name)[1]]}", WEB_FILE_{i}, {len(data)}}},')
lines += ["", "static const WebFile WEB_FILES[] = {", *entries, "};", ""]
content = "\n".join(lines)

os.makedirs(out_dir, exist_ok=True)
# Only rewrite when something changed, so unchanged pages don't force a rebuild.
old = None
if os.path.exists(out_path):
    with open(out_path) as f:
        old = f.read()
if old != content:
    with open(out_path, "w") as f:
        f.write(content)
    print(f"embed_web: {len(files)} files from web/ -> {out_path}")

env.Append(CPPPATH=[out_dir])  # noqa: F821
