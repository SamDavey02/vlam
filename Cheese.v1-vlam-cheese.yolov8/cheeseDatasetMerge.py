from pathlib import Path
import shutil

src = Path(".")
dst = Path("../food_segmentation_4class")

# Cheese source class 0 -> final class 2
NEW_CLASS_ID = 2

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

            # Source dataset only has class 0 = Cheese
            if int(parts[0]) != 0:
                continue

            parts[0] = str(NEW_CLASS_ID)
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

        # Prefix filenames so they cannot collide with the existing dataset
        new_stem = f"cheese_{label_file.stem}"

        new_image_name = new_stem + image_file.suffix
        new_label_name = new_stem + ".txt"

        shutil.copy2(
            image_file,
            dst_images / new_image_name
        )

        (dst_labels / new_label_name).write_text(
            "\n".join(new_lines) + "\n"
        )

        added_images += 1

    print(
        f"{split}: added {added_images} cheese images, "
        f"{added_instances} cheese instances"
    )

print("\nCheese merge complete.")
