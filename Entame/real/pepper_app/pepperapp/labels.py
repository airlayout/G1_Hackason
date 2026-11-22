"""Japanese display names for triggers, actions and COCO labels."""
from .events import SEEN_PREFIX

TRIGGER_LABELS = {
    "person_appeared": "人が現れた",
    "person_visible": "人が見えている間（追従）",
    "person_left": "人がいなくなった",
    "person_near": "人が近づいた",
    "hand_raised": "手を挙げた",
    "both_hands_up": "両手を挙げた",
    "wave": "手を振った",
}
ACTION_LABELS = {
    "look": "その方向を見る",
    "look_front": "正面を見る",
    "wave": "手を振る",
}
# YOLO (COCO) labels that are useful indoors, with Japanese names. Person is handled by the pose model.
OBJECT_LABELS_JA = {
    "chair": "椅子", "couch": "ソファ", "bed": "ベッド", "dining table": "テーブル",
    "tv": "テレビ", "laptop": "ノートPC", "cell phone": "スマホ", "book": "本",
    "bottle": "ボトル", "cup": "コップ", "backpack": "リュック", "handbag": "かばん",
    "umbrella": "傘", "potted plant": "植木鉢", "clock": "時計", "teddy bear": "ぬいぐるみ",
    "dog": "犬", "cat": "猫", "banana": "バナナ", "apple": "りんご",
}


# The 80 COCO labels YOLO11n knows. Kept here so the UI does not load a model to list them.
COCO_LABELS = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush",
)
OBJECT_CHOICES = tuple(label for label in COCO_LABELS if label != "person")


def object_label(label: str) -> str:
    ja = OBJECT_LABELS_JA.get(label)
    return f"{ja}（{label}）" if ja else label


def trigger_label(trigger: str) -> str:
    if trigger.startswith(SEEN_PREFIX):
        return f"見つけた: {object_label(trigger[len(SEEN_PREFIX):])}"
    return TRIGGER_LABELS.get(trigger, trigger)


def trigger_options(object_labels) -> dict[str, str]:
    """Display label -> trigger key, for the rule editor."""
    keys = list(TRIGGER_LABELS) + [SEEN_PREFIX + label for label in object_labels]
    return {trigger_label(key): key for key in keys}
