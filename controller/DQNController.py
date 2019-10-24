import keras
import numpy as np


class DQNController:
    def __init__(self, model_file: str):
        try:
            self.model = keras.models.load_model(model_file)
        except FileNotFoundError:
            print("Specified model file not found!")

        self.action_mappings = [-5, -2, 0, 2, 5]

    def act(self, state: np.ndarray):
        q_values = self.model.predict(state)
        action = np.argmax(q_values[0])
        return self.action_mappings[action]
