from pathlib import Path
import shutil

src = Path(".")
dst = Path("../food_segmentation_4class")

SOURCE_CLASS = 0   # Lettuce-plant
TARGET_CLASS = 3   # lettuce

for split in ["train", "valid", "test"]:
    src_images = src / split / "images"
    src_labels = src / split / "labels"

    dst_images = dst / split / "images"
    dst_labels = dst / split / "labels"

    added_images = 0
    added_instances = 0

    for label_file in src_labels.glob("*.txt"):
        new_lines = []

        for line in label_file.read_text().splitlines():
            if not line.strip():
                continue

            parts = line.split()

            if int(parts[0]) != SOURCE_CLASS:
                continue

            # Remap Lettuce-plant -> lettuce
            parts[0] = str(TARGET_CLASS)

            # Preserve all segmentation polygon coordinates
            new_lines.append(" ".join(parts))
            added_instances += 1

        if not new_lines:
            continue

        image_file = None

        for ext in [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]:
            candidate = src_images / f"{label_file.stem}{ext}"

            if candidate.exists():
                image_file = candidate
                break

        if image_file is None:
            print(f"WARNING: missing image for {label_file.name}")
            continue

        # Prefix filenames to avoid collisions
        new_stem = f"lettuce_{label_file.stem}"

        shutil.copy2(
            image_file,
            dst_images / f"{new_stem}{image_file.suffix}"
        )

        (dst_labels / f"{new_stem}.txt").write_text(
            "\n".join(new_lines) + "\n"
        )

        added_images += 1

    print(
        f"{split}: added {added_images} images, "
        f"{added_instances} lettuce instances"
    )

print("\nLettuce merge complete.")
