from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
from pathlib import Path

GROUNDINGDINO_ROOT = Path(__file__).resolve().parent.parent
MODEL_CONFIG = GROUNDINGDINO_ROOT / "groundingdino" / "config" / "GroundingDINO_SwinT_OGC.py"
MODEL_WEIGHTS = GROUNDINGDINO_ROOT / "weights" / "groundingdino_swint_ogc.pth"
model = load_model(str(MODEL_CONFIG), str(MODEL_WEIGHTS))
# IMAGE_PATH = GROUNDINGDINO_ROOT / "test" / "pic" / "test.jpg"
IMAGE_PATH = GROUNDINGDINO_ROOT / "test" / "pic" / "corridor.png"



TEXT_PROMPT = "fire extinguisher"
# TEXT_PROMPT = "chair . male"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25
 
image_source, image = load_image(str(IMAGE_PATH))
 
boxes, logits, phrases = predict(
    model=model,
    image=image,
    caption=TEXT_PROMPT,
    box_threshold=BOX_TRESHOLD,
    text_threshold=TEXT_TRESHOLD
)
 
annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
cv2.imwrite("annotated_image.jpg", annotated_frame)
