#!/usr/bin/env python3
"""Check that every example is built and documented.

The examples live in examples/<section>/<name>/, with the source <name>.cpp. Every one
- is built by its section: examples/<section>/CMakeLists.txt has BITRL_ADD_EXAMPLE(<name> ...);
- has a page <name>.md whose first line is
      \\page bitrl_example_<name> <title>
- is linked from examples/examples.md.

Usage: ci/check_example_docs.py [path-to-bitrl]
"""
import re
import sys
from pathlib import Path


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1])
    examples = root / "examples"
    index = (examples / "examples.md").read_text()
    linked = set(re.findall(r"\\ref bitrl_example_([a-z0-9_]+)\b", index))

    errors = []
    names = []
    for source in sorted(examples.glob("*/*/*.cpp")):
        directory = source.parent
        if source.stem != directory.name:
            continue
        name = directory.name
        section = directory.parent
        names.append(name)

        cmake = section / "CMakeLists.txt"
        built = cmake.exists() and re.search(rf"BITRL_ADD_EXAMPLE\({name}\b", cmake.read_text())
        if not built:
            errors.append(f"{cmake.relative_to(root)} does not build {name}: add BITRL_ADD_EXAMPLE({name})")

        page = directory / f"{name}.md"
        if not page.exists():
            errors.append(f"{name} has no page: add {page.relative_to(root)}")
            continue
        text = page.read_text()
        first = text.splitlines()[0] if text else ""
        if not re.fullmatch(rf"\\page bitrl_example_{name} \S.*", first):
            errors.append(f"{page.relative_to(root)}: the first line must be "
                          f"'\\page bitrl_example_{name} <title>', not '{first}'")
        if name not in linked:
            errors.append(f"examples/examples.md does not link to \\ref bitrl_example_{name}")

    duplicates = sorted({n for n in names if names.count(n) > 1})
    for name in duplicates:
        errors.append(f"two examples are called {name}: the executables and pages would clash")

    # Anchors inside a page, such as bitrl_example_<name>_mqtt, are fine to link to
    for name in sorted(linked - set(names)):
        if not any(name.startswith(n + "_") for n in names):
            errors.append(f"examples/examples.md links to bitrl_example_{name}, which does not exist")

    for error in errors:
        print(f"error: {error}")
    if not errors:
        print(f"{len(names)} examples are built, have a page and are linked from examples.md")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
