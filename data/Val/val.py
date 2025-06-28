import os
import shutil
from pathlib import Path

def create_validation_set():
    # Define paths
    base_path = Path(__file__).parent.parent
    dataset_path = base_path / 'dataset'
    val_path = base_path / 'Val'
    
    # Create validation directories if they don't exist
    val_images_dir = val_path / 'images'
    val_labels_dir = val_path / 'labels'
    val_images_dir.mkdir(exist_ok=True)
    val_labels_dir.mkdir(exist_ok=True)

    # Get all image files
    image_files = list((dataset_path / 'images').glob('*.jpeg'))
    
    # Group images by object type
    object_groups = {}
    for img_path in image_files:
        # Extract the object type from filename (everything before the number)
        object_type = ''.join(filter(lambda x: not x.isdigit(), img_path.stem))
        if object_type not in object_groups:
            object_groups[object_type] = []
        object_groups[object_type].append(img_path)

    # For each object type, move the 30th image and its corresponding label
    for object_type, images in object_groups.items():
        images.sort()  # Ensure consistent order
        if len(images) >= 30:
            # Get the 30th image
            img_to_move = images[29]  # 29 because of 0-based indexing
            
            # Construct the path for corresponding label file
            label_path = dataset_path / 'labels' / f"{img_to_move.stem}.txt"
            
            # Move image and label if they exist
            if img_to_move.exists() and label_path.exists():
                # Move image
                shutil.move(
                    str(img_to_move),
                    str(val_images_dir / img_to_move.name)
                )
                
                # Move label
                shutil.move(
                    str(label_path),
                    str(val_labels_dir / label_path.name)
                )
                print(f"Moved {img_to_move.name} and its label to validation set")
            else:
                print(f"Warning: Could not find image or label for {img_to_move.name}")

if __name__ == "__main__":
    create_validation_set()
    print("Validation set creation completed!")
