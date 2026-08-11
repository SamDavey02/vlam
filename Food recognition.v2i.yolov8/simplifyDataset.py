from pathlib import Path
import shutil

src = Path(".")
dst = Path("../food_segmentation_4class")

# Original dataset ID -> our new class ID
class_map = {
    50: 0,  # sliced tomatoe
    49: 1,  # sliced bread
    16: 2,  # cheese
    17: 2,  # cheese slice
    31: 3,  # lettuce
}

class_names = [
    "sliced_tomato",
    "sliced_bread",
    "cheese",
    "lettuce",
]

for split in ["train", "valid", "test"]:

    src_images = src / split / "images"
    src_labels = src / split / "labels"

    dst_images = dst / split / "images"
    dst_labels = dst / split / "labels"

    dst_images.mkdir(parents=True, exist_ok=True)
    dst_labels.mkdir(parents=True, exist_ok=True)

    kept_images = 0
    kept_instances = [0, 0, 0, 0]

    for label_file in src_labels.glob("*.txt"):

        new_lines = []

        for line in label_file.read_text().splitlines():

            if not line.strip():
                continue

            parts = line.split()
            old_class = int(parts[0])

            if old_class in class_map:

                new_class = class_map[old_class]

                # Change ONLY the class ID.
                # All segmentation polygon coordinates remain unchanged.
                parts[0] = str(new_class)

                new_lines.append(" ".join(parts))
                kept_instances[new_class] += 1

        # Skip images that don't contain one of our target classes
        if not new_lines:
            continue

        image_file = None

        for ext in [".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG"]:

            candidate = src_images / f"{label_file.stem}{ext}"

            if candidate.exists():
                image_file = candidate
                break

        if image_file is None:
            print(f"WARNING: image missing for {label_file.name}")
            continue

        shutil.copy2(
            image_file,
            dst_images / image_file.name
        )

        (dst_labels / label_file.name).write_text(
            "\n".join(new_lines) + "\n"
        )

        kept_images += 1

    print(f"\n{split.upper()}")
    print("-" * 40)
    print(f"Images: {kept_images}")

    for i, name in enumerate(class_names):
        print(f"{name:15}: {kept_instances[i]} instances")


# Create YOLO dataset configuration
data_yaml = """path: .
train: train/images
val: valid/images
test: test/images

nc: 4

names:
  0: sliced_tomato
  1: sliced_bread
  2: cheese
  3: lettuce
"""

(dst / "data.yaml").write_text(data_yaml)

print("\nDataset created:")
print(dst.resolve())

print("\nClasses:")
for i, name in enumerate(class_names):
    print(f"{i}: {name}")
