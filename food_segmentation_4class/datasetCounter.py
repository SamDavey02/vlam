from pathlib import Path
from collections import Counter

names = {
    0: "sliced_tomato",
    1: "sliced_bread",
    2: "cheese",
    3: "lettuce",
}

for split in ["train", "valid", "test"]:
    instances = Counter()
    images = Counter()

    for label_file in (Path(split) / "labels").glob("*.txt"):
        seen = set()

        for line in label_file.read_text().splitlines():
            if not line.strip():
                continue

            cid = int(line.split()[0])
            instances[cid] += 1
            seen.add(cid)

        # Count an image once per class, even if it
        # contains multiple instances of that class
        for cid in seen:
            images[cid] += 1

    print(f"\n{split.upper()}")
    print("-" * 50)

    for cid, name in names.items():
        print(
            f"{name:15} "
            f"images={images[cid]:4}  "
            f"instances={instances[cid]:4}"
        )
