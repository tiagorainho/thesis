from typing import List, Tuple


class Segment:
    path: List[Tuple[float]]

    def __init__(self, path) -> None:
        self.path = path

    def is_empty(self):
        return len(self.path) == 0

    def segment_pairs(self):
        return [v for v in zip(self.path, self.path[1:])]
