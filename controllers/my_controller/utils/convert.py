import os
import random
import shutil

# Define paths
dataset_path = "./fire_dataset/fire_dataset"
train_path = "./datasets/dataset/train"
valid_path = "./datasets/dataset/valid"

# Create category mapping
class_mapping = {"fire_images": 0, "non_fire_images": 1}

# Get images from both classes
fire_images = [os.path.join(dataset_path, "fire_images", f) for f in os.listdir(os.path.join(dataset_path, "fire_images")) if f.endswith(".png")]
non_fire_images = [os.path.join(dataset_path, "non_fire_images", f) for f in os.listdir(os.path.join(dataset_path, "non_fire_images")) if f.endswith(".png")]

print(f"🔥 Fire Images Found: {len(fire_images)}")
print(f"🚫 Non-Fire Images Found: {len(non_fire_images)}")

# Combine and shuffle
all_images = [(img, class_mapping["fire_images"]) for img in fire_images] + [(img, class_mapping["non_fire_images"]) for img in non_fire_images]
random.shuffle(all_images)

# Split dataset (80% train, 20% validation)
split = int(0.8 * len(all_images))
train_data = all_images[:split]
valid_data = all_images[split:]

# Function to move files and create YOLO labels
def move_data(data, split_type):
    # Ensure directories exist
    os.makedirs(f"./datasets/dataset/{split_type}/images", exist_ok=True)
    os.makedirs(f"./datasets/dataset/{split_type}/labels", exist_ok=True)

    for img_path, label in data:
        filename = os.path.basename(img_path)
        
        # Copy image to train/valid set
        shutil.copy(img_path, os.path.join(f"./datasets/dataset/{split_type}/images", filename))
        
        # Generate YOLO label with a full-image bounding box
        with open(os.path.join(f"./datasets/dataset/{split_type}/labels", filename.replace(".png", ".txt")), "w") as f:
            f.write(f"{label} 0.5 0.5 1.0 1.0\n")  # Class, X_center, Y_center, Width, Height (normalized)

# Move train and validation data
move_data(train_data, "train")
move_data(valid_data, "valid")

print("✅ Dataset organized for YOLO classification!")