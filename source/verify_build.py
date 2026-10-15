"""Compare rebuilt Linux inference executables with the frozen release ZIPs."""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORT = Path(__file__).with_name("build_validation.json")


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def elf_image(data: bytes) -> dict:
    if data[:6] != b"\x7fELF\x02\x01":
        raise ValueError("Expected a little-endian ELF64 executable")
    machine = struct.unpack_from("<H", data, 18)[0]
    entry, phoff, shoff = struct.unpack_from("<QQQ", data, 24)
    phsize, phcount, shsize, shcount, shstrings = struct.unpack_from("<HHHHH", data, 54)
    ph_types = [struct.unpack_from("<I", data, phoff + i * phsize)[0] for i in range(phcount)]
    table = [struct.unpack_from("<IIQQQQIIQQ", data, shoff + i * shsize) for i in range(shcount)]
    names = data[table[shstrings][4] : table[shstrings][4] + table[shstrings][5]]
    allocated = {}
    for name, kind, flags, address, offset, size, link, info, align, entsize in table:
        key = names[name : names.index(b"\0", name)].decode("ascii")
        if flags & 2 and key != ".note.gnu.build-id":
            allocated[key] = {
                "type": kind,
                "flags": flags,
                "address": address,
                "size": size,
                "alignment": align,
                "sha256": sha256(data[offset : offset + size]) if kind != 8 else None,
            }
    if ".text" not in allocated or ".rodata" not in allocated:
        raise ValueError("Missing executable code or model data")
    return {"machine": machine, "entry": entry, "static": machine == 62 and 2 not in ph_types and 3 not in ph_types,
            "allocated_sections": allocated}


def source_hashes() -> dict:
    result = {}
    for path in sorted((ROOT / "source").rglob("*")):
        if path.suffix in {".cpp", ".hpp", ".c", ".h", ".ps1", ".py", ".z"}:
            data = path.read_bytes()
            if path.suffix != ".z":
                data = data.replace(b"\r\n", b"\n")
            result[path.relative_to(ROOT).as_posix()] = sha256(data)
    return result


def verify() -> dict:
    index = json.loads((ROOT / "VERSION_INDEX.json").read_text(encoding="utf-8"))
    checks = []
    for version in index["versions"]:
        name = version["name"]
        archive_path = ROOT / version["package"]
        if sha256(archive_path.read_bytes()) != version["package_sha256"]:
            raise ValueError(f"Release ZIP checksum mismatch: {name}")
        with zipfile.ZipFile(archive_path) as archive:
            member = archive.getinfo("bin/estimate")
            if (archive.namelist() != ["bin/estimate"] or archive.testzip() is not None
                    or member.create_system != 3 or (member.external_attr >> 16) & 0o777 != 0o755):
                raise ValueError(f"Invalid ZIP structure or permissions: {name}")
            original = archive.read(member)
        if sha256(original) != version["binary_sha256"]:
            raise ValueError(f"Release binary checksum mismatch: {name}")
        binary_path = ROOT / "build" / name / "linux" / "estimate"
        rebuilt = binary_path.read_bytes()
        old_image, new_image = elf_image(original), elf_image(rebuilt)
        matches = old_image == new_image
        checks.append({
            "variant": name,
            "source": version["source"],
            "source_provenance": version["source_provenance"],
            "package": version["package"],
            "package_sha256": version["package_sha256"],
            "original_binary_sha256": sha256(original),
            "rebuilt_binary": binary_path.relative_to(ROOT).as_posix(),
            "rebuilt_binary_sha256": sha256(rebuilt),
            "whole_file_identical": original == rebuilt,
            "static_x86_64": new_image["static"],
            "entry_and_allocated_sections_identical": matches,
            "rebuilt_elf": new_image,
        })
        if not matches or not new_image["static"]:
            raise ValueError(f"Rebuilt ELF differs in entry or allocated sections: {name}")
    return {
        "status": "passed",
        "compiler": "Zig 0.13.0 (enforced by source/build.ps1)",
        "scope": "Source checksums, release ZIP integrity and permissions, static ELF format, entry and all SHF_ALLOC sections",
        "ignored_allocated_sections": [".note.gnu.build-id"],
        "source_hash_normalization": "CRLF to LF for text; model binary unchanged",
        "source_sha256": source_hashes(),
        "runtime_retested": False,
        "rebuilt_binaries_official_server_tested": False,
        "versions": checks,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-report", action="store_true", help="Update the tracked validation report")
    args = parser.parse_args()
    report = verify()
    if args.write_report:
        REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    else:
        expected = json.loads(REPORT.read_text(encoding="utf-8"))
        if expected["source_sha256"] != report["source_sha256"]:
            raise ValueError("Source checksums differ from the recorded snapshot")
    for item in report["versions"]:
        print(f"{item['variant']}: static x86-64 ELF, entry and all allocated sections match the release")


if __name__ == "__main__":
    main()
