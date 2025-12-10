import numpy as np

class TinyMLP:
    def __init__(self, W1, b1, W2, b2):
        # W1: (H, D) / b1: (H,)
        # W2: (C, H) / b2: (C,)
        self.W1 = np.array(W1, dtype=np.float32)
        self.b1 = np.array(b1, dtype=np.float32)
        self.W2 = np.array(W2, dtype=np.float32)
        self.b2 = np.array(b2, dtype=np.float32)

    def forward(self, x: np.ndarray) -> np.ndarray:
        # x: (D,)
        h = np.maximum(0, self.W1 @ x + self.b1)  # ReLU
        y = self.W2 @ h + self.b2                 # logits
        return y

    def predict_class(self, x: np.ndarray) -> int:
        logits = self.forward(x)
        return int(np.argmax(logits))
    
def class_to_twist(cmd_class: int):
    linear = 0.0
    angular = 0.0

    if cmd_class == 0:      # Forward
      linear = 0.15
    elif cmd_class == 1:    # Backward
      linear = -0.15
    elif cmd_class == 2:    # Left
      angular = 0.6
    elif cmd_class == 3:    # Right
      angular = -0.6
    elif cmd_class == 4:    # Stop
        linear = 0.0
        angular = 0.0

    return linear, angular

from geometry_msgs.msg import Twist
import numpy as np

# 예시용 랜덤 weight (학습 이후에 교체)
D = N_FEATURES
H = 8
C = 5

W1 = np.random.randn(H, D) * 0.1
b1 = np.zeros(H)
W2 = np.random.randn(C, H) * 0.1
b2 = np.zeros(C)

mlp = TinyMLP(W1, b1, W2, b2)

# 2) text_to_cmd 대신 MLP를 쓰는 함수
def text_to_cmd_mlp(self, text: str):
    x = text_to_features(text)        # (D,)
    cmd_class = mlp.predict_class(x)  # 0~4

    linear, angular = class_to_twist(cmd_class)

    rospy.loginfo(f"MLP 예측: class={cmd_class}, v={linear:.2f}, w={angular:.2f}")

    with self.lock:
        self.current_cmd.linear.x = linear
        self.current_cmd.angular.z = angular
        
while not rospy.is_shutdown():
    text = self.listen_once()
    if not text:
        continue
    self.text_to_cmd_mlp(text)