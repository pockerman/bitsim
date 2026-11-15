#!/usr/bin/env python3
"""Check that every example is documented.

Every examples/example_N/ has an example_N.md whose first line is
    \\page bitrl_example_N BitRL Example N: <title>
and examples/examples.md links to it. The examples that have no page yet are
listed in UNDOCUMENTED; when you write a page for one, remove it from there.

Usage: ci/check_example_docs.py [path-to-bitrl]
"""
import re
import sys
from pathlib import Path

UNDOCUMENTED = {5, 6, 7, 8, 9, 10, 15, 16, 17, 18, 19, 20, 23}


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parents[1])
    examples = root / "examples"
    index = (examples / "examples.md").read_text()
    linked = {int(n) for n in re.findall(r"\\ref bitrl_example_(\d+)\b", index)}

    errors = []
    numbers = sorted(int(d.name.split("_")[1]) for d in examples.glob("example_*") if d.is_dir())
    for n in numbers:
        page = examples / f"example_{n}" / f"example_{n}.md"
        if n in UNDOCUMENTED:
            if page.exists():
                errors.append(f"{page.relative_to(root)} exists: remove {n} from UNDOCUMENTED in {Path(__file__).name}")
            if n in linked:
                errors.append(f"examples/examples.md links to example {n}, which has no page")
            continue
        if not page.exists():
            errors.append(f"example_{n} has no page: add {page.relative_to(root)}")
            continue
        first = page.read_text().splitlines()[0] if page.read_text() else ""
        if not re.fullmatch(rf"\\page bitrl_example_{n} BitRL Example {n}: \S.*", first):
            errors.append(f"{page.relative_to(root)}: the first line must be "
                          f"'\\page bitrl_example_{n} BitRL Example {n}: <title>', not '{first}'")
        if n not in linked:
            errors.append(f"examples/examples.md does not link to \\ref bitrl_example_{n}")

    for n in sorted(linked - set(numbers)):
        errors.append(f"examples/examples.md links to example {n}, which does not exist")

    for error in errors:
        print(f"error: {error}")
    if not errors:
        print(f"{len(numbers) - len(UNDOCUMENTED)} example pages are fine, {len(UNDOCUMENTED)} examples have no page yet")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
