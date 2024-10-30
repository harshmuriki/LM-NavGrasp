import clip
import torch
import numpy as np
import torchvision
from PIL import Image
import torch.nn.functional as F
import matplotlib.pyplot as plt
import torchvision.transforms as transforms


def plot_image_with_boxes(image, boxes, labels=None):
    fig, ax = plt.subplots(1)
    ax.imshow(transforms.ToPILImage()(image))
    for i, box in enumerate(boxes):
        x_min, y_min, x_max, y_max = box
        rect = plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, fill=False, edgecolor='green')
        ax.add_patch(rect)
        if labels is not None:
            label_size = len(labels) * 10
            ax.text(x_min + (x_max - x_min) / 2 - label_size / 2, y_min, labels[i], fontsize=10, verticalalignment='top', color='black')

    plt.tight_layout()
    plt.axis('off')
    plt.show()


def get_coordinates_of_object_from_query(model, preprocessor, query, img_tensor, boxes, width, height):
    max_similarity = -np.inf
    max_rel_box = None
    model.eval()
    for box in boxes:
        x_min, y_min , x_max, y_max = box
        x_min, y_min , x_max, y_max = int(x_min.item()), int(y_min.item()), min(width, int(x_max.item())), min(height, int(y_max.item()))

        if x_min > x_max or x_min > width:
            continue

        if y_min > y_max or y_min > height:
            continue

        cropped_image = img_tensor[:,y_min:y_max+1,x_min:x_max+1]
        if not torch.prod(torch.tensor(cropped_image.shape)):
            continue

        cropped_image = preprocessor(transforms.functional.to_pil_image(cropped_image)).unsqueeze(0).to(device)
        tokenized_query = clip.tokenize(query).to(device)

        image_emb = model.encode_image(cropped_image)
        text_emb = model.encode_text(tokenized_query)
        image_emb = F.normalize(image_emb, p=2, dim=-1)
        text_emb = text_emb.mean(dim=0, keepdim=True)
        text_emb = F.normalize(text_emb, p=2, dim=-1)

        similarity = torch.dot(image_emb.view(-1), text_emb.view(-1))
        if similarity > max_similarity:
            max_similarity = similarity
            max_rel_box = box
    return max_rel_box.cpu().numpy()


queries = [
    ['a yellow ball']
]
labels = ['a yellow ball']
device = "cuda" if torch.cuda.is_available() else "cpu"

clip_model, preprocessor = clip.load("ViT-B/32", device=device)
img = Image.open("output_folder/image_0246.png").convert("RGB")
# img = preprocessor(Image.open("output_folder/image_0246.png")).unsqueeze(0).to(device)


faster_rcnn = torchvision.models.detection.fasterrcnn_resnet50_fpn(
    weights=torchvision.models.detection.FasterRCNN_ResNet50_FPN_Weights.DEFAULT).cuda()
faster_rcnn.eval()

# =========================== BEGIN ===========================

transform = transforms.ToTensor()
img_tensor = transform(img).cuda()
# pass the image through the model
'''transformed = faster_rcnn.transform([img_tensor])[0]
features = faster_rcnn.backbone(transformed.tensors)
boxes = faster_rcnn.rpn(transformed, features)[0][0]'''
second_boxes = faster_rcnn(img_tensor.unsqueeze(0))[0]['boxes'].detach()
width, height = img.size


all_boxes = []
for query, label in zip(queries, labels):
    pred_box = get_coordinates_of_object_from_query(clip_model, preprocessor, query, img_tensor, second_boxes, width, height)
    all_boxes.append(pred_box)
    plot_image_with_boxes(img_tensor, [pred_box], [label])